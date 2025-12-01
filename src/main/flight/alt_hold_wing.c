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

#include "platform.h"

#ifdef USE_WING

#include "math.h"

#ifdef USE_ALTITUDE_HOLD

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "rx/rx.h"
#include "pg/autopilot.h"

#include "alt_hold.h"

static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ);

typedef struct {
    bool isActive;
    float targetAltitudeCm;
} altHoldState_t;

altHoldState_t altHold;

LOCAL_UNUSED_FUNCTION static void altHoldReset(void)
{
    resetAltitudeControl();
    altHold.targetAltitudeCm = getAltitudeCm();
    altHold.targetVelocity = 0.0f;
}

void altHoldInit(void)
{
    altHold.isActive = false;
    altHold.deadband = 0;
    altHold.allowStickAdjustment = false;
    altHold.maxVelocity = 0;
    altHoldReset();
}

void updateAltHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    altHoldProcessTransitions();

    if (altHold.isActive) {
        altHoldUpdate();
    }
}

bool isAltHoldActive(void) {
    return altHold.isActive;
}

#endif // USE_ALTITUDE_HOLD
#endif // USE_WING
