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

#ifdef USE_ALTITUDE_HOLD

#include "build/debug.h"

#include "common/maths.h"

#include "config/config.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "rx/rx.h"

#include "pg/autopilot.h"

#include "alt_hold.h"

static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ); // 0.01s

typedef struct {
    bool isActive;
    float targetAltitudeCm;
    float maxVelocity;
    float targetVelocity;
    float deadband;
    bool allowStickAdjustment;
} altHoldState_t;

static altHoldState_t altHold;

static void altHoldReset(void)
{
    resetAltitudeControl();
    // Init target to current altitude to avoid D-kick on activation
    altHold.targetAltitudeCm = getAltitudeCmControl();
    altHold.targetVelocity = 0.0f;
}

void altHoldInit(void)
{
    altHold.isActive = false;
    altHold.deadband = altHoldConfig()->deadband / 100.0f;
    altHold.allowStickAdjustment = altHoldConfig()->deadband > 0;
    altHold.maxVelocity = altHoldConfig()->climbRate * 10.0f; // CLI value 50 = 500 cm/s
    altHoldReset();
}

static void altHoldProcessTransitions(void)
{
    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        if (!altHold.isActive) {
            altHoldReset();
            altHold.isActive = true;
        }
    } else {
        if (altHold.isActive) {
            resetAltitudeControl();
        }
        altHold.isActive = false;
    }
}

static void altHoldUpdateTargetAltitude(void)
{
    // Wing uses pitch stick to adjust target altitude:
    // pull back (negative rcCommand) = climb = increase target
    // push forward (positive rcCommand) = descend = decrease target
    float stickFactor = 0.0f;

    if (altHold.allowStickAdjustment) {
        const float pitchStick = rcCommand[PITCH];
        const float stickRange = 500.0f;
        const float threshold = altHold.deadband * stickRange;

        if (pitchStick < -threshold) {
            // Pull back → climb (increase target altitude)
            stickFactor = scaleRangef(pitchStick, -stickRange, -threshold, 1.0f, 0.0f);
        } else if (pitchStick > threshold) {
            // Push forward → descend (decrease target altitude)
            stickFactor = scaleRangef(pitchStick, threshold, stickRange, 0.0f, -1.0f);
        }
    }

    if (failsafeIsActive()) {
        // Descend during failsafe, faster when higher
        // Wing-safe descent: gentler than multirotor, capped at 2x climb rate
        stickFactor = -(0.5f + constrainf(getAltitudeCmControl() / 5000.0f, 0.0f, 1.5f));
    }

    altHold.targetVelocity = stickFactor * altHold.maxVelocity;

    // Prevent target from drifting too far from current altitude
    if (fabsf(getAltitudeCmControl() - altHold.targetAltitudeCm) < altHold.maxVelocity * 1.0f) {
        altHold.targetAltitudeCm += altHold.targetVelocity * taskIntervalSeconds;
    }
}

static void altHoldUpdate(void)
{
    if (altHoldConfig()->climbRate) {
        altHoldUpdateTargetAltitude();
    }
    altitudeControl(altHold.targetAltitudeCm, taskIntervalSeconds, altHold.targetVelocity, altHold.maxVelocity);
}

void updateAltHold(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    altHoldProcessTransitions();

    if (altHold.isActive) {
        altHoldUpdate();
    }
}

bool isAltHoldActive(void)
{
    return altHold.isActive;
}

#endif // USE_ALTITUDE_HOLD
#endif // USE_WING
