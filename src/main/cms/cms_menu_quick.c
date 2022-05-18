/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

//
// Main menu structure and support functions
//

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CMS

#include "cms/cms.h"
#include "cms/cms_types.h"

// Sub menus

#include "cms/cms_menu_imu.h"
#include "cms/cms_menu_blackbox.h"
#include "cms/cms_menu_failsafe.h"
#include "cms/cms_menu_firmware.h"
#include "cms/cms_menu_ledstrip.h"
#include "cms/cms_menu_misc.h"
#include "cms/cms_menu_osd.h"
#include "cms/cms_menu_power.h"
#include "cms/cms_menu_saveexit.h"
#include "cms/cms_menu_main.h"

#ifdef USE_PERSISTENT_STATS
#include "cms/cms_menu_persistent_stats.h"
#endif

// VTX supplied menus

#include "cms/cms_menu_vtx_common.h"

#include "common/printf.h"
#include "fc/controlrate_profile.h"
#include "config/config.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"

#include "cms_menu_quick.h"

// Features

static controlRateConfig_t rateProfile;
static uint8_t rateProfileIndex;

static const void *quickMenuOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    rateProfileIndex = getCurrentControlRateProfileIndex();
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    // fill variables
    return NULL;
}

static const void *cmsx_RateProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return NULL;
}


static const char * const osdTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};

static const OSD_Entry menuMainEntries[] =
{
    {"-- QUICK --",  OME_Label, NULL, NULL},

    { "THR LIM TYPE",OME_TAB,    NULL, &(OSD_TAB_t)   { &rateProfile.throttle_limit_type, THROTTLE_LIMIT_TYPE_COUNT - 1, osdTableThrottleLimitType} },
    { "THR LIM %",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.throttle_limit_percent, 25,  100,  1} },
#if defined(USE_VTX_CONTROL)
#if defined(USE_VTX_RTC6705) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    {"VTX", OME_Funcall, cmsSelectVtx, NULL},
#endif
#endif // VTX_CONTROL
    {"MAIN",     OME_Submenu,  NULL, &cmsx_menuMain},
    { "EXIT",            OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT},
    { "SAVE&EXIT",       OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVE},
    {NULL, OME_END, NULL, NULL},
};

CMS_Menu cmsx_menuQuick = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUQUICK",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = quickMenuOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = menuMainEntries,
};

#endif
