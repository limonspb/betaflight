/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING
#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"
#include "rx/rx.h"
#include "pg/autopilot.h"
#include "sensors/acceleration.h"

#include "gps_rescue.h"

// Enums

typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_FLY_HOME,
    RESCUE_DESCEND_TO_LOITER,
    RESCUE_LOITER,
    RESCUE_DESCEND_TO_LAND,
    RESCUE_APPROACH,
    RESCUE_WAIT_FOR_COURSE,
    RESCUE_LAST_COURSE_ADJ,
    RESCUE_LAND,
    RESCUE_DO_NOTHING,
    RESCUE_ABORT,
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_NO_HOME_POINT,
} rescueFailure_e;

// State

typedef struct {
    rescuePhase_e phase;
    rescueFailure_e failure;
    bool isAvailable;
    float currentAltitudeCm;
    float distanceToHomeM;
    float directionToHomeDdeg;   // decidegrees
    float maxAltitudeCm;
    float returnAltitudeCm;
    float descentDistanceM;
    float disarmThreshold;
    float taskIntervalS;
} rescueState_t;

static rescueState_t rescue;

// PID state (own, separate from alt_hold/pos_hold)

#define ALT_P_SCALE   0.1f
#define ALT_I_SCALE   0.01f
#define ALT_D_SCALE   0.1f
#define ALT_I_WINDUP  15.0f     // degrees
#define ALT_I_RESET_M 15.0f     // meters

#define COG_P_SCALE   0.1f
#define COG_I_SCALE   0.001f
#define COG_D_SCALE   0.1f
#define COG_I_WINDUP  15.0f     // degrees
#define COG_I_RESET   20.0f     // degrees

static float altI = 0.0f;
static float prevAltErr = 0.0f;
static pt2Filter_t altDLpf;

static float cogI = 0.0f;
static float prevCogErr = 0.0f;
static float calcRoll = 0.0f;
static float prevPitchDeg = 0.0f;
static timeUs_t pitchJumpTime = 0;

float gpsRescueAngle[RP_AXIS_COUNT] = { 0, 0 };

// Helpers

static bool checkAvailable(void)
{
    static timeUs_t prevTimeUs = 0;
    static int8_t lowSatCount = 0;
    static bool lowSats = false;
    static bool noFix = false;
    const timeUs_t now = micros();

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME)) {
        return false;
    }

    const timeDelta_t dt = cmpTimeUs(now, prevTimeUs);
    if (dt < 1000000) {
        return !(noFix || lowSats);
    }
    prevTimeUs = now;

    if (!STATE(GPS_FIX)) {
        noFix = true;
        return false;
    }
    noFix = false;

    lowSatCount = constrain(lowSatCount + ((gpsSol.numSat < gpsRescueConfig()->minSats) ? 1 : -1), 0, 2);
    lowSats = (lowSatCount >= 2);
    return !lowSats;
}

static void updateReturnAltitude(void)
{
    if (!ARMING_FLAG(ARMED)) {
        rescue.maxAltitudeCm = 0.0f;
        return;
    }
    rescue.maxAltitudeCm = fmaxf(rescue.currentAltitudeCm, rescue.maxAltitudeCm);

    const float initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
    const float returnMinCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
    rescue.returnAltitudeCm = fmaxf(returnMinCm, rescue.maxAltitudeCm + initialClimbCm);
    rescue.descentDistanceM = fminf(0.5f * rescue.distanceToHomeM, (float)gpsRescueConfig()->descentDistanceM);
}

static void sensorUpdate(void)
{
    rescue.currentAltitudeCm = getAltitudeCmControl();
    rescue.distanceToHomeM = GPS_distanceToHomeCm / 100.0f;
    rescue.directionToHomeDdeg = GPS_directionToHome; // decidegrees
    rescue.taskIntervalS = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);
}

static void disarmOnImpact(void)
{
    if (acc.accMagnitude > rescue.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescue.phase = RESCUE_IDLE;
    }
}

// PID controllers

static void rescueAttainPosition(float desiredCourseDdeg, float desiredAltCm)
{
    if (rescue.phase == RESCUE_IDLE) {
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        setThrottleCut(false);
        return;
    }
    if (rescue.phase == RESCUE_DO_NOTHING) {
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        setThrottleCut(true);
        return;
    }

    const gpsRescueConfig_t *cfg = gpsRescueConfig();
    const float dt = rescue.taskIntervalS;

    // Altitude PID
    const float altErrM = (rescue.currentAltitudeCm - desiredAltCm) / 100.0f;

    const float aP = ALT_P_SCALE * cfg->altP * altErrM;

    altI += ALT_I_SCALE * cfg->altI * altErrM * dt;
    altI = constrainf(altI, -ALT_I_WINDUP, ALT_I_WINDUP);
    if (fabsf(altErrM) > ALT_I_RESET_M) {
        altI = 0.0f;
    }

    float aD = (prevAltErr - altErrM) / dt;
    prevAltErr = altErrM;
    aD *= ALT_D_SCALE * cfg->altD;
    aD = pt2FilterApply(&altDLpf, aD);

    const float pitchDeg = aP + altI - aD;

    // Course PID (GPS rate only)
    static uint16_t gpsStamp = 0;

    if (gpsHasNewData(&gpsStamp)) {
        float cogErr = (desiredCourseDdeg - (float)gpsSol.groundCourse) / 10.0f;
        if (cogErr > 180.0f) cogErr -= 360.0f;
        if (cogErr < -180.0f) cogErr += 360.0f;

        // Anti-oscillation: reject sudden large COG jumps
        if (fabsf(cogErr - prevCogErr) > 270.0f) {
            cogErr = prevCogErr;
        }

        const float gpsInt = fmaxf(getGpsDataIntervalSeconds(), 0.01f);

        const float cP = COG_P_SCALE * cfg->cogP * cogErr;

        cogI += COG_I_SCALE * cfg->cogI * cogErr * gpsInt;
        cogI = constrainf(cogI, -COG_I_WINDUP, COG_I_WINDUP);
        if (fabsf(cogErr) > COG_I_RESET) {
            cogI = 0.0f;
        }

        float cD = (cogErr - prevCogErr) / gpsInt;
        prevCogErr = cogErr;
        cD *= COG_D_SCALE * cfg->cogD;

        calcRoll = cP + cogI + cD;
        calcRoll = constrainf(calcRoll, -(float)cfg->maxRescueAngle, (float)cfg->maxRescueAngle);
    }

    // Altitude priority — suppress roll during large pitch changes
    float rollDeg = calcRoll;
    if (fabsf(pitchDeg - prevPitchDeg) > 45.0f) {
        pitchJumpTime = micros();
    }
    prevPitchDeg = pitchDeg;

    if (cmpTimeUs(micros(), pitchJumpTime) < 1000000 || fabsf(pitchDeg) > 45.0f) {
        rollDeg = 0.0f;
        cogI = 0.0f;
    }

    // Roll-pitch mix — banking lift loss compensation
    float pitchOut = pitchDeg;
    if (cfg->rollPitchMix > 0) {
        pitchOut -= fabsf((float)cfg->rollPitchMix / 100.0f * rollDeg);
    }

    // Output in centidegrees for pid.c integration
    gpsRescueAngle[AI_ROLL] = rollDeg * 100.0f;
    gpsRescueAngle[AI_PITCH] = constrainf(pitchOut, -(float)cfg->maxRescueAngle, (float)cfg->maxRescueAngle) * 100.0f;

    // Throttle cut during landing phases
    const bool landingPhase = (rescue.phase == RESCUE_LAND ||
                               rescue.phase == RESCUE_DO_NOTHING);
    setThrottleCut(landingPhase);

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(pitchOut * 100));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 1, lrintf(rollDeg * 100));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(rescue.currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(desiredAltCm));
}

// Sanity checks

static void performSanityChecks(void)
{
    static timeUs_t prevTimeUs = 0;
    static int8_t lowSatSec = 0;
    static int8_t doNothingSec = 0;
    const timeUs_t now = micros();

    if (rescue.phase == RESCUE_IDLE) {
        rescue.failure = RESCUE_HEALTHY;
        lowSatSec = 0;
        doNothingSec = 0;
        prevTimeUs = 0;
        return;
    }

    const bool hardFailsafe = !isRxReceivingSignal();

    if (rescue.failure != RESCUE_HEALTHY) {
        rescue.phase = RESCUE_DO_NOTHING;
        switch (gpsRescueConfig()->sanityChecks) {
        case RESCUE_SANITY_ON:
            rescue.phase = RESCUE_ABORT;
            break;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescue.phase = RESCUE_ABORT;
            }
            break;
        case RESCUE_SANITY_OFF:
        default:
            if (gpsRescueConfig()->allowArmingWithoutFix && !STATE(GPS_FIX_HOME) && hardFailsafe) {
                rescue.phase = RESCUE_ABORT;
            }
            break;
        }
    }

    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_CRASH_PROTECTION);
        rescue.phase = RESCUE_IDLE;
    }

    if (!gpsIsHealthy()) {
        rescue.failure = RESCUE_GPSLOST;
    }

    const timeDelta_t dTime = cmpTimeUs(now, prevTimeUs);
    if (dTime < 1000000) {
        return;
    }
    prevTimeUs = now;

    lowSatSec += (!STATE(GPS_FIX) || (gpsSol.numSat < gpsRescueConfig()->minSats)) ? 1 : -1;
    lowSatSec = constrain(lowSatSec, 0, 10);
    if (lowSatSec >= 10) {
        rescue.failure = RESCUE_LOWSATS;
    }

    if (rescue.phase == RESCUE_DO_NOTHING) {
        doNothingSec = MIN(doNothingSec + 1, 20);
        if (doNothingSec >= 20) {
            rescue.phase = RESCUE_ABORT;
        }
    }
}

// Main state machine

void gpsRescueInit(void)
{
    const float gain = pt2FilterGain(0.75f, HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ));
    pt2FilterInit(&altDLpf, gain);
    rescue.phase = RESCUE_IDLE;
    rescue.failure = RESCUE_HEALTHY;
    rescue.maxAltitudeCm = 0.0f;
}

void gpsRescueUpdate(void)
{
    float desiredCourse = rescue.directionToHomeDdeg;
    float desiredAlt = rescue.returnAltitudeCm;
    static float descendAlt = 0.0f;
    static float desiredLandingCourse = 0.0f;
    static bool landingCourseWasBad = false;
    static timeUs_t loiterTime = 0;
    static timeUs_t waitCourseTime = 0;
    static timeUs_t lastCourseAdjTime = 0;
    static timeUs_t landTime = 0;

    const gpsRescueConfig_t *cfg = gpsRescueConfig();
    const bool hasLoiter = cfg->loiterAltM > 0 && cfg->loiterSeconds > 0;

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescue.phase = RESCUE_IDLE;
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescue.phase == RESCUE_IDLE) {
        rescue.phase = RESCUE_INITIALIZE;
    }

    sensorUpdate();
    rescue.isAvailable = checkAvailable();

    switch (rescue.phase) {
    case RESCUE_IDLE:
        updateReturnAltitude();
        break;

    case RESCUE_INITIALIZE: {
        // Reset PID state (must happen here, before phase transitions)
        altI = 0.0f;
        prevAltErr = 0.0f;
        cogI = 0.0f;
        prevCogErr = 0.0f;
        calcRoll = 0.0f;
        prevPitchDeg = 0.0f;
        pitchJumpTime = micros();
        const float gain = pt2FilterGain(0.75f, HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ));
        pt2FilterInit(&altDLpf, gain);
        rescue.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        setThrottleCut(false);

        landingCourseWasBad = false;
        if (!STATE(GPS_FIX_HOME)) {
            rescue.failure = RESCUE_NO_HOME_POINT;
        } else {
            if (rescue.distanceToHomeM < 5.0f && isBelowLandingAltitude()) {
                rescue.phase = RESCUE_ABORT;
            } else {
                if (rescue.distanceToHomeM < cfg->minStartDistM) {
                    rescue.returnAltitudeCm = fmaxf(1000.0f, rescue.currentAltitudeCm + cfg->initialClimbM * 100.0f);
                    rescue.descentDistanceM = 0.5f * cfg->minStartDistM;
                }
                rescue.phase = RESCUE_FLY_HOME;
            }
        }
        break;
    }

    case RESCUE_FLY_HOME:
        if (rescue.distanceToHomeM <= rescue.descentDistanceM) {
            descendAlt = rescue.currentAltitudeCm;
            rescue.phase = hasLoiter ? RESCUE_DESCEND_TO_LOITER : RESCUE_DESCEND_TO_LAND;
        }
        break;

    case RESCUE_DESCEND_TO_LOITER:
        // Stepwise descent
        descendAlt -= cfg->descendRate * rescue.taskIntervalS;
        desiredAlt = descendAlt;
        if (rescue.currentAltitudeCm / 100.0f < cfg->loiterAltM) {
            loiterTime = micros();
            rescue.phase = RESCUE_LOITER;
        }
        break;

    case RESCUE_LOITER: {
        desiredAlt = cfg->loiterAltM * 100.0f;
        const timeDelta_t elapsed = cmpTimeUs(micros(), loiterTime);
        if (elapsed >= 0 && (timeUs_t)elapsed >= (timeUs_t)cfg->loiterSeconds * 1000000) {
            descendAlt = rescue.currentAltitudeCm;
            rescue.phase = RESCUE_DESCEND_TO_LAND;
        }
        break;
    }

    case RESCUE_DESCEND_TO_LAND:
        // Stepwise descent
        descendAlt -= cfg->descendRate * rescue.taskIntervalS;
        desiredAlt = descendAlt;
        if (rescue.currentAltitudeCm / 100.0f < cfg->landingAltM) {
            descendAlt = cfg->landingAltM * 100.0f;
            rescue.phase = RESCUE_APPROACH;
        }
        break;

    case RESCUE_APPROACH:
        // Fly away from home, then turn back to align landing
        desiredAlt = descendAlt;
        desiredCourse = fmodf(rescue.directionToHomeDdeg + 1800.0f, 3600.0f);
        if (rescue.distanceToHomeM > cfg->landingApproachDistM) {
            waitCourseTime = micros();
            rescue.phase = RESCUE_WAIT_FOR_COURSE;
        }
        break;

    case RESCUE_WAIT_FOR_COURSE: {
        // Wait until heading aligns toward home (or 60s timeout)
        desiredAlt = descendAlt;
        float cErr = (desiredCourse - gpsSol.groundCourse) / 10.0f;
        if (cErr > 180.0f) cErr -= 360.0f;
        if (cErr < -180.0f) cErr += 360.0f;

        const bool timedOut = cmpTimeUs(micros(), waitCourseTime) >= 60000000;
        if ((fabsf(cErr) < 20.0f && landingCourseWasBad) || timedOut) {
            lastCourseAdjTime = micros();
            descendAlt = rescue.currentAltitudeCm;
            rescue.phase = RESCUE_LAST_COURSE_ADJ;
        } else {
            landingCourseWasBad = true;
        }
        break;
    }

    case RESCUE_LAST_COURSE_ADJ: {
        desiredAlt = descendAlt;
        float cErr = (desiredCourse - gpsSol.groundCourse) / 10.0f;
        if (cErr > 180.0f) cErr -= 360.0f;
        if (cErr < -180.0f) cErr += 360.0f;

        const bool waited = cmpTimeUs(micros(), lastCourseAdjTime) >= 3000000;
        if (waited || fabsf(cErr) < 3.0f || rescue.distanceToHomeM < 15.0f) {
            desiredLandingCourse = desiredCourse;
            landTime = micros();
            descendAlt = rescue.currentAltitudeCm;
            rescue.phase = RESCUE_LAND;
        }
        break;
    }

    case RESCUE_LAND:
        // Continue descending
        descendAlt -= cfg->descendRate * rescue.taskIntervalS;
        if (rescue.currentAltitudeCm < descendAlt + 100) {
            descendAlt = rescue.currentAltitudeCm + 100;
        }
        desiredAlt = descendAlt;
        desiredCourse = desiredLandingCourse;

        // Disarm on impact after 2s
        if (cmpTimeUs(micros(), landTime) >= 2000000) {
            disarmOnImpact();
        }
        break;

    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_FAILSAFE);
        rescue.phase = RESCUE_IDLE;
        break;

    case RESCUE_DO_NOTHING:
        disarmOnImpact();
        break;

    default:
        break;
    }

    performSanityChecks();
    rescueAttainPosition(desiredCourse, desiredAlt);

    DEBUG_SET(DEBUG_RTH, 0, rescue.phase);
    DEBUG_SET(DEBUG_RTH, 1, lrintf(rescue.distanceToHomeM));
    DEBUG_SET(DEBUG_RTH, 2, lrintf(rescue.currentAltitudeCm / 100));
    DEBUG_SET(DEBUG_RTH, 3, lrintf(desiredAlt / 100));
}

// Roll-yaw mix for coordinated turns
float gpsRescueGetYawRate(void)
{
    return -(float)gpsRescueConfig()->rollYawMix / 100.0f * gpsRescueAngle[AI_ROLL] / 100.0f;
}

float gpsRescueGetImuYawCogGain(void)
{
    return 10.0f; // wing does not use IMU heading, constant neutral value
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsAvailable(void)
{
    return rescue.isAvailable;
}

bool gpsRescueIsDisabled(void)
{
    return !STATE(GPS_FIX_HOME);
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    return false; // wing does not use compass
}
#endif

#endif // USE_GPS_RESCUE

#endif // USE_WING
