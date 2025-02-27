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

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct autopilotConfig_s {
    uint16_t throttle;
    uint16_t throttleMin;
    uint16_t throttleMax;

    uint8_t altitudeP;
    uint8_t altitudeI;
    uint8_t altitudeD;

    uint8_t altitudeDLpfHz;

    uint8_t cogP;
    uint8_t cogI;
    uint8_t cogD;

    uint8_t maxRoll;
    uint8_t maxPitch;
} autopilotConfig_t;

PG_DECLARE(autopilotConfig_t, autopilotConfig);

#endif // USE_WING
