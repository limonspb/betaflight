/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/io.h"

#include "fc/rc_controls.h"

#include "rx/rx.h"
#include "rx/sbus.h"

#include "sensors/gyro.h"

#include "flight/imu.h"

#if defined(USE_HEADTRACKER)
static int16_t yawOffset = 1800;

#define SHIMMY_PERIOD 500000 // 500ms
#define SHIMMY_AMP 100       // Shimmy aplitude in deg/s
#define SHIMMY_COUNT 5

void headtrackerYawReset(void)
{
    yawOffset = (1800 - attitude.values.yaw + 3600) % 3600;
}

bool headtrackerInit(void)
{
    IO_t headtrackerIO = IOGetByTag(rxConfig()->headtracker_ioTag);

    if (headtrackerIO) {
        IOInit(headtrackerIO, OWNER_HEADTRACKER, 0);
        IOConfigGPIO(headtrackerIO, IOCFG_IPU);
    }

    return true;
}

void taskHeadtracker(uint32_t currentTime)
{
    UNUSED(currentTime);
    int16_t angles[3];
    uint16_t channels[3];
    IO_t headtrackerIO = IOGetByTag(rxConfig()->headtracker_ioTag);
    float yawGyro = gyroGetFilteredDownsampled(YAW);
    static int8_t shimmyCount = 0;
    static uint32_t shimmyStartTime = 0;

    // Check for a head shimmy to reset the yaw
    if (cmpTimeUs(currentTime, shimmyStartTime) > SHIMMY_PERIOD) {
        shimmyCount = 0;
    }

    bool odd = shimmyCount % 2;
    if ((odd && (yawGyro > SHIMMY_AMP)) || (!odd && (yawGyro < -SHIMMY_AMP))) {
        shimmyStartTime = currentTime;
        shimmyCount++;
    }

    if ((headtrackerIO && !IORead(headtrackerIO)) || (shimmyCount == SHIMMY_COUNT)) {
        shimmyCount = 0;
        headtrackerYawReset();
    }
    // Use the SBus output to send attitude information for headtracking
    angles[ROLL] = attitude.values.roll + 1800;
    angles[PITCH] = attitude.values.pitch + 1800;
    angles[YAW] = (attitude.values.yaw + yawOffset) % 3600;

    if (0 != rxConfig()->headtracker_max_angle) {
        const int newValueMin = 1800 - rxConfig()->headtracker_max_angle * 10;
        const int newValueMax = 1800 + rxConfig()->headtracker_max_angle * 10;
        for (int channel = 0; channel < 3; channel++) {
            angles[channel] = scaleRange(angles[channel], newValueMin, newValueMax, 0, 3600);
            angles[channel] = constrain(angles[channel], 0, 3600);
        }
    }

    for (int channel = 0; channel < 3; channel++) {
        channels[channel] = angles[channel] * 2048 / 3600;
    }

    sbusTx(channels, 3);
}
#endif



