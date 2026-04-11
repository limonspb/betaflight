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

#include <math.h>

#include "platform.h"

#ifdef USE_WING

#ifdef USE_POSITION_HOLD

#include "build/debug.h"

#include "common/maths.h"

#include "config/config.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/gps.h"

#include "rx/rx.h"

#include "pg/pos_hold.h"

#include "pos_hold.h"

typedef struct {
    bool isEnabled;
    bool sticksWereActive;
    float deadband;
} posHoldState_t;

static posHoldState_t posHold;

void posHoldInit(void)
{
    posHold.isEnabled = false;
    posHold.deadband = posHoldConfig()->deadband / 100.0f;
    posHold.sticksWereActive = false;
}

static void posHoldCheckSticks(void)
{
    if (failsafeIsActive()) {
        setSticksActiveStatus(false);
        return;
    }

    const bool sticksDeflected = getRcDeflectionAbs(FD_ROLL) > posHold.deadband;
    setSticksActiveStatus(sticksDeflected);

    // When sticks return to center, update target position
    if (posHold.sticksWereActive && !sticksDeflected) {
        resetPositionControl(POSHOLD_TASK_RATE_HZ);
    }
    posHold.sticksWereActive = sticksDeflected;
}

void updatePosHold(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            resetPositionControl(POSHOLD_TASK_RATE_HZ);
            posHold.isEnabled = true;
        }
    } else {
        if (posHold.isEnabled) {
            autopilotAngle[AI_ROLL] = 0.0f;
            setSticksActiveStatus(false);
        }
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled) {
        posHoldCheckSticks();
        if (isAutopilotInControl()) {
            positionControl();
        }
    }
}

bool posHoldFailure(void)
{
    return FLIGHT_MODE(POS_HOLD_MODE) && !gpsIsHealthy();
}

#endif // USE_POSITION_HOLD

#endif // USE_WING
