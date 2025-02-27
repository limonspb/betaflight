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
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "autopilot.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.01f
#define ALTITUDE_D_SCALE  0.01f
#define POSITION_P_SCALE  0.0012f
#define POSITION_I_SCALE  0.0001f
#define POSITION_D_SCALE  0.0015f

float autopilotAngle[RP_AXIS_COUNT];

typedef struct autopilotState_s {
    gpsLocation_t targetLocation;
    bool sticksActive;
    float cogI;
    float altitudeI;
    pt2Filter_t altitudeDLpf;
    float altHoldTaskInvervalS;
} autopilotState_t;

static autopilotState_t apState = {
    .cogI = 0.0f,
    .altitudeI = 0.0f,
    .sticksActive = false,
    .altHoldTaskInvervalS = 0.0f,
};

void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz)
{
    apState.targetLocation = *initialTargetLocation;
    apState.sticksActive = false;
    apState.cogI = 0.0f;
    UNUSED(initialTargetLocation);
    UNUSED(taskRateHz);
}

void autopilotInit(void)
{
    apState.sticksActive = false;
}

void resetAltitudeControl (void) {
    apState.altitudeI = 0.0f;
}

void initAltitudeFiltersIfChanged(float taskIntervalS)
{
    if (apState.altHoldTaskInvervalS == taskIntervalS) {
        return;
    }

    apState.altHoldTaskInvervalS = taskIntervalS;
    const float altitudeDLpfCutoff = autopilotConfig()->altitudeDLpfHz;
    const float altitudeDLpfGain = pt2FilterGain(altitudeDLpfCutoff, taskIntervalS);
    pt2FilterInit(&apState.altitudeDLpf, altitudeDLpfGain);
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep)
{
    initAltitudeFiltersIfChanged(taskIntervalS);
    const float altitudeErrorCm = targetAltitudeCm - getAltitudeCm();
    UNUSED(targetAltitudeCm);
    UNUSED(taskIntervalS);
    UNUSED(targetAltitudeStep);
}

void setSticksActiveStatus(bool areSticksActive)
{
    UNUSED(areSticksActive);
}

bool positionControl(void)
{
    return false;
}

bool isBelowLandingAltitude(void)
{
    return false;
}

float getAutopilotThrottle(void)
{
    return 0.0f;
}

bool isAutopilotInControl(void)
{
    return false;
}

#endif // USE_WING
