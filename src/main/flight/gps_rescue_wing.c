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

#ifdef USE_WING

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"
#include "config/config.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"

#include "gps_rescue.h"

typedef enum {
    RESCUE_IDLE, // 0
    RESCUE_INITIALIZE, // 1
    RESCUE_FLY_HOME, // 2
    RESCUE_DESCEND_TO_LOITER, // 3
    RESCUE_LOITER, // 4
    RESCUE_DESCEND_TO_LAND, // 5
    RESCUE_APPROACH_MANEUVRE, // 6
    RESCUE_WAIT_FOR_CORUSE, // 7
    RESCUE_PREPARE_TO_LAND, // 8
    RESCUE_LAST_COURSE_ADJUSTMENT, // 9
    RESCUE_LAND, // 10
    RESCUE_DO_NOTHING,
    RESCUE_ABORT,
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASH_FLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT
} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    float targetVelocityCmS;
    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    float descentDistanceM;
    int8_t secondsFailing;
    float disarmThreshold;
    float velocityITermAccumulator;
    float velocityPidCutoff;
    float velocityPidCutoffModifier;
    float velocityItermAttenuator;
    float velocityItermRelax;
} rescueIntent_s;

typedef struct {
    float currentAltitudeCm;
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float altitudeDataIntervalSeconds;
    float gpsRescueTaskIntervalSeconds;
    float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
} rescueState_s;

#define GPS_RESCUE_MAX_ANGULAR_ITERM     1500    // max iterm value for pitch in degrees * 100
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f  // yaw error must be less than this to enter fly home phase, and to pitch during descend()

float       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
bool        magForceDisable = false;
static bool newGPSData = false;
static pt2Filter_t throttleDLpf;
static pt1Filter_t velocityDLpf;
static pt3Filter_t velocityUpsampleLpf;

rescueState_s rescueState;

void gpsRescueInit(void)
{
    rescueState.sensor.gpsRescueTaskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);

    float cutoffHz, gain;
    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt2FilterInit(&throttleDLpf, gain);

    cutoffHz = gpsRescueConfig()->pitchCutoffHz / 100.0f;
    rescueState.intent.velocityPidCutoff = cutoffHz;
    rescueState.intent.velocityPidCutoffModifier = 1.0f;
    gain = pt1FilterGain(cutoffHz, 1.0f);
    pt1FilterInit(&velocityDLpf, gain);

    cutoffHz *= 4.0f;
    gain = pt3FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt3FilterInit(&velocityUpsampleLpf, gain);
}

/*
 If we have new GPS data, update home heading if possible and applicable.
*/
void gpsRescueNewGpsData(void)
{
    newGPSData = true;
}

static void rescueStart(void)
{
    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
}

// Things that need to run when GPS Rescue is enabled, and while armed, but while there is no Rescue in place
static void setReturnAltitude(void)
{
    // Hold maxAltitude at zero while disarmed, but if set_home_point_once is true, hold maxAlt until power cycled
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.intent.maxAltitudeCm = 0.0f;
        return;
    }

    // While armed, but not during the rescue, update the max altitude value
    rescueState.intent.maxAltitudeCm = fmaxf(rescueState.sensor.currentAltitudeCm, rescueState.intent.maxAltitudeCm);

    if (newGPSData) {
        // set the target altitude to current values, so there will be no D kick on first run
        rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;

        // Intended descent distance for rescues that start outside the minStartDistM distance
        // Set this to the user's intended descent distance, but not more than half the distance to home to ensure some fly home time
        rescueState.intent.descentDistanceM = fminf(0.5f * rescueState.sensor.distanceToHomeM, gpsRescueConfig()->descentDistanceM);

        const float initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                // climb above current altitude, but always return at least initial height above takeoff point, in case current altitude was negative
                rescueState.intent.returnAltitudeCm = fmaxf(initialClimbCm, rescueState.sensor.currentAltitudeCm + initialClimbCm);
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + initialClimbCm;
                break;
        }
    }
}

static void rescueAttainPosition(float desiredCourseDecidegrees, float desiredAltitudeCm)
{
    // runs at 100hz, but only updates RPYT settings when new GPS Data arrives and when not in idle phase.
    static float altI = 0.0f;
    static float courseI = 0.0f;
    static float previousAltitudeError = 0.0f;
    static float previousCourseError = 0.0f;
    static float calculatedRollDegrees = 0.0f;
    static float calculatedPitchDegrees = 0.0f;
    static float calculatedPitchDegreesPrevious = 0.0f;
    timeUs_t lastPitchJumpTime = 0;

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // values to be returned when no rescue is active
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return;
    case RESCUE_INITIALIZE:
        // Initialize internal variables each time GPS Rescue is started
        altI = 0.0f;
        previousAltitudeError = 0.0f;
        previousCourseError = 0.0f;
        rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
        rescueState.sensor.imuYawCogGain = 1.0f;
        return;
    case RESCUE_DO_NOTHING:
        // 20s of slow descent for switch induced sanity failures to allow time to recover
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return;
     default:
        break;
    }

    // currentAltitudeCm is updated at TASK_GPS_RESCUE_RATE_HZ
    const float altitudeError = (rescueState.sensor.currentAltitudeCm - desiredAltitudeCm) / 100.0f;
    // height above target in metres (negative means too low)
    // at the start, the target starts at current altitude plus one step.  Increases stepwise to intended value.

    // P component
    const float altP = 0.1f * gpsRescueConfig()->ap_wing_alt_p * altitudeError;

    // I component
    altI += 0.01f * gpsRescueConfig()->ap_wing_alt_i * altitudeError * rescueState.sensor.altitudeDataIntervalSeconds;
    altI = constrainf(altI, -1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM, 1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM);
    if (ABS(altitudeError) > 15.0f) { // turn off I-term if altitude is too far from the desired one
        altI = 0.0f;
    }
    // up to 20% increase in throttle from I alone

    // D component is error based, so includes positive boost when climbing and negative boost on descent
    float altD = ((previousAltitudeError - altitudeError) / rescueState.sensor.altitudeDataIntervalSeconds);
    previousAltitudeError = altitudeError;
    // apply user's throttle D gain
    altD *= 0.1f * gpsRescueConfig()->ap_wing_alt_d;
    // smooth
    altD = pt2FilterApply(&throttleDLpf, altD);

    calculatedPitchDegreesPrevious = calculatedPitchDegrees;
    calculatedPitchDegrees = altP + altI - altD;


    //DEBUG_SET(DEBUG_WING_RTH, 0, lrintf(altP * 100)); // throttle D before lowpass smoothing
    //DEBUG_SET(DEBUG_WING_RTH, 1, lrintf(altI * 100)); // throttle D before lowpass smoothing
    //DEBUG_SET(DEBUG_WING_RTH, 2, lrintf(altD * 100)); // throttle D before lowpass smoothing
    //DEBUG_SET(DEBUG_WING_RTH, 3, lrintf(gpsRescueAngle[AI_PITCH])); // throttle D before lowpass smoothing

    if (newGPSData) {
        // course PID
        float courseError = (desiredCourseDecidegrees - gpsSol.groundCourse) / 10.0f;
        if (courseError > 180.0f) {
            courseError -= 360.0f;
        }

        if (courseError < -180.0f) {
            courseError += 360.0f;
        }

        if (ABS(courseError - previousCourseError) > 270.0) {
            courseError = previousCourseError;
        }

        //DEBUG_SET(DEBUG_WING_RTH, 0, lrintf(gpsSol.groundCourse)); // throttle D before lowpass smoothing // degrees
        //DEBUG_SET(DEBUG_WING_RTH, 1, lrintf(rescueState.sensor.directionToHome)); // throttle D before lowpass smoothing // degrees
        //DEBUG_SET(DEBUG_WING_RTH, 2, lrintf(courseError)); // throttle D before lowpass smoothing // degrees
        const float courseP = 0.1f * courseError * gpsRescueConfig()->ap_wing_cog_p;

        courseI += 0.001f * gpsRescueConfig()->ap_wing_cog_i * courseError * rescueState.sensor.gpsDataIntervalSeconds;
        courseI = constrainf(courseI, -1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM / 100.0f, 1.0f * GPS_RESCUE_MAX_ANGULAR_ITERM / 100.0f);
        if (ABS(courseError) > 20.0f) { // turn off I-term if cource error is > 20 degrees
            courseI = 0.0f;
        }

        float courseD = ((courseError - previousCourseError) / rescueState.sensor.gpsDataIntervalSeconds);
        previousCourseError = courseError;
        // apply user's throttle D gain
        courseD *= 0.1f * gpsRescueConfig()->ap_wing_cog_d;

        calculatedRollDegrees = courseP + courseI + courseD;
        calculatedRollDegrees = constrainf(calculatedRollDegrees, -gpsRescueConfig()->maxRescueAngle, gpsRescueConfig()->maxRescueAngle);

        //DEBUG_SET(DEBUG_WING_RTH, 0, lrintf(calculatedRollDegrees)); // throttle D before lowpass smoothing

        //DEBUG_SET(DEBUG_WING_RTH, 6, lrintf(courseD)); // throttle D before lowpass smoothing
        //DEBUG_SET(DEBUG_WING_RTH, 7, lrintf(gpsRescueAngle[AI_ROLL])); // throttle D before lowpass smoothing

    }

    //DEBUG_SET(DEBUG_WING_RTH, 4, lrintf(calculatedRollDegrees * 100.0f));
    //DEBUG_SET(DEBUG_WING_RTH, 5, lrintf(calculatedPitchDegrees * 100.0f));

    float rollDegrees = calculatedRollDegrees;
    if (ABS(calculatedPitchDegrees - calculatedPitchDegreesPrevious) > 45.0) {
        lastPitchJumpTime = micros();
    }

    if (lastPitchJumpTime - micros() < 1000000 || ABS(calculatedPitchDegrees) > 45.0) {
        rollDegrees = 0.0f;
        courseI = 0.0f;
    }

    gpsRescueAngle[AI_ROLL] = rollDegrees * 100.0f;
    gpsRescueAngle[AI_PITCH] = calculatedPitchDegrees * 100.0f - ABS((float)gpsRescueConfig()->ap_wing_roll_pitch_mix / 100.0f * gpsRescueAngle[AI_ROLL]);
    gpsRescueAngle[AI_PITCH] = constrainf(gpsRescueAngle[AI_PITCH], -gpsRescueConfig()->maxRescueAngle * 100.0f, gpsRescueConfig()->maxRescueAngle * 100.0f);

}


static void performSanityChecks(void)
{
    static timeUs_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static int8_t secondsDoingNothing; // Limit on doing nothing
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize these variables each time a GPS Rescue is started
        previousTimeUs = currentTimeUs;
        secondsLowSats = 0;
        secondsDoingNothing = 0;
    }

    // Handle events that set a failure mode to other than healthy.
    // Disarm via Abort when sanity on, or for hard Rx loss in FS_ONLY mode
    // Otherwise allow 20s of semi-controlled descent with impact disarm detection
    const bool hardFailsafe = !isRxReceivingSignal();

    if (rescueState.failure != RESCUE_HEALTHY) {
        // Default to 20s semi-controlled descent with impact detection, then abort
        rescueState.phase = RESCUE_DO_NOTHING;

        switch(gpsRescueConfig()->sanityChecks) {
        case RESCUE_SANITY_ON:
            rescueState.phase = RESCUE_ABORT;
            break;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescueState.phase = RESCUE_ABORT;
            }
            break;
        default:
            // even with sanity checks off,
            // override when Allow Arming without Fix is enabled without GPS_FIX_HOME and no Control link available.
            if (gpsRescueConfig()->allowArmingWithoutFix && !STATE(GPS_FIX_HOME) && hardFailsafe) {
                rescueState.phase = RESCUE_ABORT;
            }
        }
    }

    // Crash detection is enabled in all rescues.  If triggered, immediately disarm.
    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_CRASH_PROTECTION);
        rescueStop();
    }

    // Check if GPS comms are healthy
    // ToDo - check if we have an altitude reading; if we have Baro, we can use Landing mode for controlled descent without GPS
    if (!rescueState.sensor.healthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc) will be checked at 1Hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        return;
    }
    previousTimeUs = currentTimeUs;

    // checks that we are getting closer to home.
    // if the quad is stuck, or if GPS data packets stop, there will be no change in distance to home
    // we can't use rescueState.sensor.currentVelocity because it will be held at the last good value if GPS data updates stop
    if (rescueState.phase == RESCUE_FLY_HOME) {
        // TODO: verify we are getting closer
        // need to keep in mind that wing not nessessary walways getting closer. For example it can be circling around the home point
        // or it can do a slow turn after GPS rescue was triggered
    }

    secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    secondsLowSats = constrain(secondsLowSats, 0, 10);

    if (secondsLowSats == 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }

    // These conditions ignore sanity mode settings, and apply in all rescues, to handle getting stuck in a climb or descend

    switch (rescueState.phase) {
    case RESCUE_DO_NOTHING:
        secondsDoingNothing = MIN(secondsDoingNothing + 1, 20);
        if (secondsDoingNothing >= 20) {
            rescueState.phase = RESCUE_ABORT;
            // time-limited semi-controlled fall with impact detection
        }
        break;
    default:
        // do nothing
        break;
    }

    DEBUG_SET(DEBUG_RTH, 2, (rescueState.failure * 10 + rescueState.phase));
    DEBUG_SET(DEBUG_RTH, 3, (rescueState.intent.secondsFailing * 100 + secondsLowSats));
}

static void sensorUpdate(void)
{
    static float prevDistanceToHomeCm = 0.0f;
    const timeUs_t currentTimeUs = micros();

    static timeUs_t previousAltitudeDataTimeUs = 0;
    const timeDelta_t altitudeDataIntervalUs = cmpTimeUs(currentTimeUs, previousAltitudeDataTimeUs);
    rescueState.sensor.altitudeDataIntervalSeconds = altitudeDataIntervalUs * 0.000001f;
    previousAltitudeDataTimeUs = currentTimeUs;

    rescueState.sensor.currentAltitudeCm = getAltitudeCm();

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(rescueState.sensor.currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, rescueState.sensor.groundSpeedCmS);  // groundspeed cm/s
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, rescueState.sensor.directionToHome); // computed from current GPS position in relation to home
    rescueState.sensor.healthy = gpsIsHealthy();

    rescueState.sensor.directionToHome = GPS_directionToHome; // extern value from gps.c using current position relative to home
    rescueState.sensor.errorAngle = (attitude.values.yaw - rescueState.sensor.directionToHome) / 10.0f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (rescueState.sensor.errorAngle <= -180) {
        rescueState.sensor.errorAngle += 360;
    } else if (rescueState.sensor.errorAngle > 180) {
        rescueState.sensor.errorAngle -= 360;
    }
    rescueState.sensor.absErrorAngle = fabsf(rescueState.sensor.errorAngle);

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 4, lrintf(attitude.values.yaw));                 // estimated heading of the quad (direction nose is pointing in)
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 5, lrintf(rescueState.sensor.directionToHome));  // angle to home derived from GPS location and home position

    if (!newGPSData) {
        return;
        // GPS ground speed, velocity and distance to home will be held at last good values if no new packets
    }

    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;
    rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    rescueState.sensor.gpsDataIntervalSeconds = getGpsDataIntervalSeconds();
    // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.

    rescueState.sensor.velocityToHomeCmS = ((prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) / rescueState.sensor.gpsDataIntervalSeconds);
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    DEBUG_SET(DEBUG_ATTITUDE, 4, rescueState.sensor.velocityToHomeCmS); // velocity to home

    // when there is a flyaway due to IMU disorientation, increase IMU yaw CoG gain, and reduce max pitch angle
    if (gpsRescueConfig()->groundSpeedCmS) {
        // calculate a factor that can reduce pitch angle when flying away
        const float rescueGroundspeed = gpsRescueConfig()->imuYawGain * 100.0f; // in cm/s, imuYawGain is m/s groundspeed
        // rescueGroundspeed is effectively a normalising gain factor for the magnitude of the groundspeed error
        // a higher value reduces groundspeedErrorRatio, making the radius wider and reducing the circling behaviour

        const float groundspeedErrorRatio = fabsf(rescueState.sensor.groundSpeedCmS - rescueState.sensor.velocityToHomeCmS) / rescueGroundspeed;
        // 0 if groundspeed = velocity to home, or both are zero
        // 1 if forward velocity is zero but sideways speed is imuYawGain in m/s
        // 2 if moving backwards at imuYawGain m/s, 4 if moving backwards at 2* imuYawGain m/s, etc

        DEBUG_SET(DEBUG_ATTITUDE, 5, groundspeedErrorRatio * 100);

        rescueState.intent.velocityItermAttenuator = 4.0f / (groundspeedErrorRatio + 4.0f);
        // 1 if groundspeedErrorRatio = 0, falling to 2/3 if groundspeedErrorRatio = 2, 0.5 if groundspeedErrorRatio = 4, etc
        // limit (essentially prevent) velocity iTerm accumulation whenever there is a meaningful groundspeed error
        // this is a crude but simple way to prevent iTerm windup when recovering from an IMU error
        // the magnitude of the effect will be less at low GPS data rates, since there will be fewer multiplications per second
        // but for our purposes this should not matter much, our intent is to severely attenuate iTerm
        // if, for example, we had a 90 degree attitude error, groundspeedErrorRatio = 1, invGroundspeedError = 0.8,
        // after 10 iterations, iTerm is 0.1 of what it would have been
        // also is useful in blocking iTerm accumulation if we overshoot the landing point

        const float pitchForwardAngle = (gpsRescueAngle[AI_PITCH] > 0.0f) ? fminf(gpsRescueAngle[AI_PITCH] / 3000.0f, 2.0f) : 0.0f;
        // pitchForwardAngle is positive early in a rescue, and associates with a nose forward ground course
        // note: gpsRescueAngle[AI_PITCH] is in degrees * 100, and is halved when the IMU is 180 wrong
        // pitchForwardAngle is 0 when flat
        // pitchForwardAngle is 0.5 if pitch angle is 15 degrees (ie with rescue angle of 30 and 180deg IMU error)
        // pitchForwardAngle is 1.0 if pitch angle is 30 degrees (ie with rescue angle of 60 and 180deg IMU error)
        // pitchForwardAngle is 2.0 if pitch angle is 60 degrees and flying towards home (unlikely to be sustained at that angle)

        DEBUG_SET(DEBUG_ATTITUDE, 6, pitchForwardAngle * 100.0f);

        if (rescueState.phase != RESCUE_FLY_HOME) {
            // prevent IMU disorientation arising from drift during climb, rotate or do nothing phases, which have zero pitch angle
            // in descent, or too close, increase IMU yaw gain as pitch angle increases
            rescueState.sensor.imuYawCogGain = pitchForwardAngle;
        } else {
            rescueState.sensor.imuYawCogGain = pitchForwardAngle + fminf(groundspeedErrorRatio, 3.5f);
            // imuYawCogGain will be more positive at higher pitch angles and higher groundspeed errors
            // imuYawCogGain will be lowest (close to zero) at lower pitch angles and when flying straight towards home
        }
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, lrintf(rescueState.sensor.velocityToHomeCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(rescueState.sensor.velocityToHomeCmS));
}

// This function flashes "RESCUE N/A" in the OSD if:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a 3D fix.
// 3. GPS number of satellites is greater than or equal to the minimum configured satellite count.
// Note 1: cannot arm without the required number of sats
// hence this flashing indicates that after having enough sats, we now have below the minimum and the rescue likely would fail
// Note 2: this function does not take into account the distance from home
// The sanity checks are independent, this just provides the OSD warning
static bool checkGPSRescueIsAvailable(void)
{
    static timeUs_t previousTimeUs = 0; // Last time LowSat was checked
    const timeUs_t currentTimeUs = micros();
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static bool lowsats = false;
    static bool noGPSfix = false;
    bool result = true;

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME) || !isFixedWing()) {
        return false;
    }

    //  Things that should run at a low refresh rate >> ~1hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        if (noGPSfix || lowsats) {
            result = false;
        }
        return result;
    }

    previousTimeUs = currentTimeUs;

    if (!STATE(GPS_FIX)) {
        result = false;
        noGPSfix = true;
    } else {
        noGPSfix = false;
    }

    secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < GPS_MIN_SAT_COUNT) ? 1 : -1), 0, 2);
    if (secondsLowSats == 2) {
        lowsats = true;
        result = false;
    } else {
        lowsats = false;
    }

    return result;
}

void disarmOnImpact(void)
{
    //DEBUG_SET(DEBUG_WING_RTH, 2, lrintf(acc.accMagnitude * 100.0f));
    //DEBUG_SET(DEBUG_WING_RTH, 3, lrintf(rescueState.intent.disarmThreshold * 100.0f));
    if (acc.accMagnitude > rescueState.intent.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStop();
    }
}

void initialiseRescueValues (void)
{
    rescueState.intent.secondsFailing = 0; // reset the sanity check timer
    rescueState.intent.targetVelocityCmS = rescueState.sensor.velocityToHomeCmS; // avoid snap from D at the start
    rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
    rescueState.intent.velocityPidCutoffModifier = 1.0f; // normal velocity lowpass filter cutoff
    rescueState.intent.pitchAngleLimitDeg = 0.0f; // force pitch adjustment to zero - level mode will level out
    rescueState.intent.velocityItermAttenuator = 1.0f; // allow iTerm to accumulate normally unless constrained by IMU error or descent phase
    rescueState.intent.velocityItermRelax = 0.0f; // but don't accumulate any at the start, not until fly home
}

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
    float desiredCourseDecidegrees = rescueState.sensor.directionToHome;
    float desiredAltitudeCm = rescueState.intent.returnAltitudeCm;
    static float desiredLandingCourseDecidegrees = 0.0f;
    static bool landingCourseWasBad = false;
    static float descendAltitudeCm = 0.0f;
    static timeUs_t loiterActivatedTime = 0;
    static timeUs_t waitForCourseActivatedTime = 0;
    static timeUs_t landActivatedTime = 0;
    static timeUs_t lastCourseAdjustmentActivatedTime = 0;

    const bool isLoiterSetup = gpsRescueConfig()->ap_wing_loiter_alt > 0 && gpsRescueConfig()->ap_wing_loiter_seconds > 0;

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        rescueAttainPosition(desiredCourseDecidegrees, desiredAltitudeCm); // Initialise basic parameters when a Rescue starts (can't initialise sensor data reliably)
        performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    // Will now be in RESCUE_INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    sensorUpdate(); // always get latest GPS and Altitude data, update ascend and descend rates

    rescueState.isAvailable = checkGPSRescueIsAvailable();

    //DEBUG_SET(DEBUG_WING_RTH, 0, lrintf(rescueState.phase));

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // in Idle phase = NOT in GPS Rescue
        // update the return altitude and descent distance values, to have valid settings immediately they are needed
        setReturnAltitude();
        break;
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE:
        landingCourseWasBad = false;
        // Things that should be done at the start of a Rescue
        if (!STATE(GPS_FIX_HOME)) {
            // we didn't get a home point on arming
            rescueState.failure = RESCUE_NO_HOME_POINT;
            // will result in a disarm via the sanity check system, or float around if switch induced
        } else {
            if (rescueState.sensor.distanceToHomeM < 5.0f && isBelowLandingAltitude()) {
                // attempted initiation within 5m of home, and 'on the ground' -> instant disarm, for safety reasons
                rescueState.phase = RESCUE_ABORT;
            } else {
                // attempted initiation within minimum rescue distance requires us to fly out to at least that distance
                if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minStartDistM) {
                    // climb above current height by buffer height, to at least 10m altitude
                    rescueState.intent.returnAltitudeCm = fmaxf(1000.0f, rescueState.sensor.currentAltitudeCm + (gpsRescueConfig()->initialClimbM * 100.0f));
                    // note that the pitch controller will pitch forward to fly out to minStartDistM
                    // set the descent distance to half the minimum rescue distance. which we should have moved out to in the climb phase
                    rescueState.intent.descentDistanceM = 0.5f * gpsRescueConfig()->minStartDistM;
                }
                // otherwise behave as for a normal rescue
                initialiseRescueValues ();
                rescueState.phase = RESCUE_FLY_HOME;
            }
        }
        break;
    case RESCUE_FLY_HOME:
        if (rescueState.sensor.distanceToHomeM <= gpsRescueConfig()->descentDistanceM) {
            if (isLoiterSetup) {
                rescueState.phase = RESCUE_DESCEND_TO_LOITER;
            } else {
                rescueState.phase = RESCUE_DESCEND_TO_LAND;
            }
            descendAltitudeCm = rescueState.sensor.currentAltitudeCm;
        }
        break;

    case RESCUE_DESCEND_TO_LOITER:
        descendAltitudeCm -= gpsRescueConfig()->descendRate * rescueState.sensor.gpsRescueTaskIntervalSeconds;
        desiredAltitudeCm = descendAltitudeCm;

        if (rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_loiter_alt) {
            loiterActivatedTime = micros();
            rescueState.phase = RESCUE_LOITER;
        }

        break;

    case RESCUE_LOITER:
        desiredAltitudeCm = gpsRescueConfig()->ap_wing_loiter_alt * 100.0f;

        if (micros() - loiterActivatedTime >= ((timeUs_t)gpsRescueConfig()->ap_wing_loiter_seconds) * 1000 * 1000) {
            descendAltitudeCm = rescueState.sensor.currentAltitudeCm;
            rescueState.phase = RESCUE_DESCEND_TO_LAND;
        }

        break;

    case RESCUE_DESCEND_TO_LAND:
        descendAltitudeCm -= gpsRescueConfig()->descendRate * rescueState.sensor.gpsRescueTaskIntervalSeconds;
        desiredAltitudeCm = descendAltitudeCm;

        if (rescueState.sensor.currentAltitudeCm / 100.0f < gpsRescueConfig()->ap_wing_landing_alt) {
            descendAltitudeCm = gpsRescueConfig()->ap_wing_landing_alt * 100.0f;
            rescueState.phase = RESCUE_APPROACH_MANEUVRE;
        }

        break;

    case RESCUE_APPROACH_MANEUVRE:
        desiredAltitudeCm = descendAltitudeCm;
        desiredCourseDecidegrees = - rescueState.sensor.directionToHome + 3600.0f;

        if (rescueState.sensor.distanceToHomeM > gpsRescueConfig()->ap_wing_landing_approach_dist) {
            waitForCourseActivatedTime = micros();
            rescueState.phase = RESCUE_WAIT_FOR_CORUSE;
        }

        break;

    case RESCUE_WAIT_FOR_CORUSE:
        float courseError = (desiredCourseDecidegrees - gpsSol.groundCourse) / 10.0f;
        desiredAltitudeCm = descendAltitudeCm;
        if (courseError > 180.0f) {
            courseError -= 360.0f;
        }

        if (courseError < -180.0f) {
            courseError += 360.0f;
        }

        const bool waitedTooLong = micros() - waitForCourseActivatedTime >= 60 * 1000 * 1000; // 1 minute

        if ((ABS(courseError) < 20.0f && landingCourseWasBad) || waitedTooLong) {
            lastCourseAdjustmentActivatedTime = micros();
            descendAltitudeCm = rescueState.sensor.currentAltitudeCm;
            rescueState.phase = RESCUE_LAST_COURSE_ADJUSTMENT;
        } else {
            landingCourseWasBad = true;
        }

        break;
    case RESCUE_LAST_COURSE_ADJUSTMENT:
        desiredAltitudeCm = descendAltitudeCm;

        courseError = (desiredCourseDecidegrees - gpsSol.groundCourse) / 10.0f;
        if (courseError > 180.0f) {
            courseError -= 360.0f;
        }

        if (courseError < -180.0f) {
            courseError += 360.0f;
        }

        const bool waitedEnough = micros() - lastCourseAdjustmentActivatedTime >= 3 * 1000 * 1000; // arbitrary 3 seconds
        if (waitedEnough || courseError < 3.0f || rescueState.sensor.distanceToHomeM < 15.0f) {
            desiredLandingCourseDecidegrees = desiredCourseDecidegrees;
            landActivatedTime = micros();
            descendAltitudeCm = rescueState.sensor.currentAltitudeCm;
            rescueState.phase = RESCUE_LAND;
        }

        break;

    case RESCUE_LAND:
        descendAltitudeCm -= gpsRescueConfig()->descendRate * rescueState.sensor.gpsRescueTaskIntervalSeconds;

        if (rescueState.sensor.currentAltitudeCm < descendAltitudeCm + 100) {
            descendAltitudeCm = rescueState.sensor.currentAltitudeCm + 100;
        }

        desiredAltitudeCm = descendAltitudeCm;
        desiredCourseDecidegrees = desiredLandingCourseDecidegrees;

        const bool checkImpact = micros() - landActivatedTime >= 2 * 1000 * 1000; // 2 seconds

        if (checkImpact) {
            disarmOnImpact();
        }

        break;

    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_FAILSAFE);
        rescueState.intent.secondsFailing = 0; // reset sanity timers so we can re-arm
        rescueStop();
        break;

    case RESCUE_DO_NOTHING:
        disarmOnImpact();
        break;

    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_RTH, 0, lrintf(rescueState.intent.maxAltitudeCm / 10.0f));

    performSanityChecks();
    rescueAttainPosition(desiredCourseDecidegrees, desiredAltitudeCm);

    newGPSData = false;
}

float gpsRescueGetYawRate(void)
{
    return - gpsRescueConfig()->ap_wing_roll_yaw_mix / 100.0f * gpsRescueAngle[AI_ROLL];
}

float gpsRescueGetImuYawCogGain(void)
{
    return rescueState.sensor.imuYawCogGain;
}

float gpsRescueGetThrottle(void)
{
    float commandedThrottle = scaleRangef(gpsRescueConfig()->throttle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    //DEBUG_SET(DEBUG_WING_RTH, 1, lrintf(commandedThrottle * 100.0f));

    // less voltage - more throttle
    float batteryThrottleFactor = 1.0f;
    if (pidRuntime.tpaSpeed.maxVoltage > 0.0f) {
        batteryThrottleFactor = getBatteryVoltageLatest() / 100.0f / pidRuntime.tpaSpeed.maxVoltage;
        batteryThrottleFactor = constrainf(batteryThrottleFactor, 0.0f, 1.0f);
    }
    commandedThrottle = commandedThrottle / batteryThrottleFactor;

    DEBUG_SET(DEBUG_WING_RTH, 2, lrintf(commandedThrottle * 100.0f));

    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_WING_RTH, 3, lrintf(commandedThrottle * 100.0f));

    // Nose down - less throttle, nose up - more throttle depending on pitch angle and thrust to weight ratio
    const float twr = pidRuntime.tpaSpeed.twr;
    DEBUG_SET(DEBUG_WING_RTH, 4, lrintf(twr * 100.0f));
    DEBUG_SET(DEBUG_WING_RTH, 5, lrintf(getSinPitchAngle() * 100.0f));


    float underSqrt = (commandedThrottle * commandedThrottle * twr - getSinPitchAngle()) / twr;

    DEBUG_SET(DEBUG_WING_RTH, 6, lrintf(underSqrt * 100.0f));

    if (underSqrt < 0.0f) {
        underSqrt = 0.0f;
    }
    commandedThrottle = sqrtf(underSqrt);
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);
    DEBUG_SET(DEBUG_WING_RTH, 7, lrintf(commandedThrottle * 100.0f));

    if (rescueState.phase == RESCUE_LAND || rescueState.phase == RESCUE_DO_NOTHING || rescueState.phase == RESCUE_LAST_COURSE_ADJUSTMENT) {
        commandedThrottle = 0.0f;
    }

    DEBUG_SET(DEBUG_WING_RTH, 0, lrintf(commandedThrottle * 100.0f));
    return commandedThrottle;
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable;
}

bool gpsRescueIsDisabled(void)
// used for OSD warning
{
    return (!STATE(GPS_FIX_HOME));
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    // Enable mag on user request, but don't use it during fly home or if force disabled
    // Note that while flying home the course over ground from GPS provides a heading that is less affected by wind
    return !(gpsRescueConfig()->useMag && rescueState.phase != RESCUE_FLY_HOME && !magForceDisable);
}
#endif
#endif

#endif // USE_WING
