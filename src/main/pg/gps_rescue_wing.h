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

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct gpsRescue_s {
    uint8_t  allowArmingWithoutFix;
    uint8_t  minSats;
    uint16_t returnAltitudeM;
    uint16_t initialClimbM;
    uint16_t descentDistanceM;
    uint16_t descendRate;           // cm/s
    uint16_t minStartDistM;
    uint8_t  maxRescueAngle;        // degrees
    uint8_t  altP, altI, altD;
    uint8_t  cogP, cogI, cogD;
    uint8_t  rollPitchMix;          // percent
    uint8_t  rollYawMix;            // percent
    uint8_t  sanityChecks;
    uint8_t  disarmThreshold;       // accelerometer threshold * 10
    uint8_t  landingAltM;
    uint16_t landingApproachDistM;
    uint8_t  loiterAltM;            // 0 = skip loiter
    uint16_t loiterSeconds;         // 0 = skip loiter
} gpsRescueConfig_t;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

#endif // USE_WING
