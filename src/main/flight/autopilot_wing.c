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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "pg/autopilot.h"

#include "alt_hold.h"
#include "autopilot.h"

// Altitude PID scale factors 
#define ALT_P_SCALE   0.1f
#define ALT_I_SCALE   0.01f
#define ALT_D_SCALE   0.1f
#define ALT_I_WINDUP_LIMIT 15.0f    // degrees, max I-term contribution
#define ALT_I_RESET_ERROR_M 15.0f   // reset I-term when altitude error exceeds this (meters)
#define ALT_D_CUTOFF_HZ 0.75f       // PT2 filter cutoff for D term

float autopilotAngle[RP_AXIS_COUNT];

static float altI = 0.0f;
static float previousAltitudeError = 0.0f;
static pt2Filter_t altDLpf;
static float throttleOut = 0.0f;
static bool sticksActive = false;

void autopilotInit(void)
{
#ifdef USE_ALTITUDE_HOLD
    const float gain = pt2FilterGain(ALT_D_CUTOFF_HZ, HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ));
    pt2FilterInit(&altDLpf, gain);
#endif
    altI = 0.0f;
    previousAltitudeError = 0.0f;
    throttleOut = 0.0f;
    sticksActive = false;
    autopilotAngle[AI_ROLL] = 0.0f;
    autopilotAngle[AI_PITCH] = 0.0f;
}

void resetAltitudeControl(void)
{
    altI = 0.0f;
    previousAltitudeError = 0.0f;
    throttleOut = 0.0f;
    autopilotAngle[AI_PITCH] = 0.0f;
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeVelCmS, float velLimitCmS)
{
    UNUSED(targetAltitudeVelCmS);
    UNUSED(velLimitCmS);

    // Altitude error in meters: positive = too high, need to pitch down
    const float altitudeErrorM = (getAltitudeCmControl() - targetAltitudeCm) / 100.0f;

    const autopilotConfig_t *cfg = autopilotConfig();

    // P term
    const float altP = ALT_P_SCALE * cfg->altitudeP * altitudeErrorM;

    // I term with windup limit and reset when far from target
    altI += ALT_I_SCALE * cfg->altitudeI * altitudeErrorM * taskIntervalS;
    altI = constrainf(altI, -ALT_I_WINDUP_LIMIT, ALT_I_WINDUP_LIMIT);
    if (fabsf(altitudeErrorM) > ALT_I_RESET_ERROR_M) {
        altI = 0.0f;
    }

    // D term on error derivative, PT2 smoothed
    float altD = (previousAltitudeError - altitudeErrorM) / taskIntervalS;
    previousAltitudeError = altitudeErrorM;
    altD *= ALT_D_SCALE * cfg->altitudeD;
    altD = pt2FilterApply(&altDLpf, altD);

    // Output pitch in degrees
    // Positive = nose down (descend when too high), Negative = nose up (climb when too low)
    // In BF convention: positive pitch = forward tilt = nose down
    const float pitchDegrees = altP + altI - altD;

    autopilotAngle[AI_PITCH] = pitchDegrees;

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(altP * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(altI * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(altD * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(targetAltitudeCm));
}

void setSticksActiveStatus(bool areSticksActive)
{
    sticksActive = areSticksActive;
}

void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
}

bool positionControl(void)
{
    return false;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCmControl() < 100.0f * autopilotConfig()->landingAltitudeM;
}

float getAutopilotThrottle(void)
{
    const autopilotConfig_t *apCfg = autopilotConfig();
    float commandedThrottle = scaleRangef(apCfg->cruiseThrottle,
        MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

    // Battery voltage compensation ()
    if (pidRuntime.tpaSpeed.maxVoltage > 0.0f) {
        float batteryFactor = getBatteryVoltageLatest() / 100.0f / pidRuntime.tpaSpeed.maxVoltage;
        batteryFactor = constrainf(batteryFactor, 0.5f, 1.0f);
        commandedThrottle /= batteryFactor;
    }
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    // Pitch-angle gravity compensation ()
    // When climbing (nose up, sinPitch < 0 in BF convention): need more throttle
    // When descending (nose down, sinPitch > 0): need less throttle
    const float twr = pidRuntime.tpaSpeed.twr;
    if (twr > 0.0f) {
        float underSqrt = (commandedThrottle * commandedThrottle * twr - getSinPitchAngle()) / twr;
        if (underSqrt < 0.0f) {
            underSqrt = 0.0f;
        }
        commandedThrottle = sqrtf(underSqrt);
    }
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    throttleOut = commandedThrottle;
    return throttleOut;
}

bool isAutopilotInControl(void)
{
    return !sticksActive;
}

#endif // USE_WING
