# Wing GPS Rescue – CLI Guide

> ## ⚠️ EXPERIMENTAL — USE AT YOUR OWN RISK
>
> This branch and the wing GPS Rescue code it contains are **work in progress and not well tested**. Behaviour may change without notice, defaults may be wrong, and there are almost certainly bugs that have not been found yet.
>
> By flying with this firmware you accept that:
> - The aircraft **may not return home** when you expect it to.
> - The aircraft **may crash, fly away, or be lost** with no warning, even if everything looks correctly configured.
> - You are solely responsible for any damage, injury, or loss caused to your aircraft, other property, or people.
>
> **Always test in a wide, open area, away from people, vehicles, buildings and other aircraft.** Have a way to take manual control at any moment, keep the aircraft within visual line of sight, and never rely on this feature as your only safety net.
>
> If you are not comfortable with the above, **do not use this firmware**.

---

This guide describes the Betaflight GPS Rescue (Return‑To‑Home) for **fixed‑wing / flying‑wing** aircraft, and every CLI parameter you can use to tune it.

Unlike a multirotor — which holds position by adjusting throttle and tilt — a wing must keep flying. So the wing GPS Rescue controls **pitch** to hold altitude, **roll** to hold heading, and uses a fixed cruise **throttle** with automatic compensation for battery sag and pitch attitude.

> **Before you start:** GPS Rescue on a wing relies on Angle mode internally. If your wing doesn't fly level in **Angle mode** at zero stick, the rescue won't fly level either. Trim the airframe correctly with **`angle_pitch_offset`** (see below) *before* tuning anything in this guide.
>
> ⚠️ **Bump `hover_throttle` first.** The default `hover_throttle` is **1275**, which is fine for a multirotor but almost always **too low for a fixed wing** to maintain level flight. Set it in the CLI right away to whatever throttle PWM your wing actually needs at cruise (commonly **1500–1700** depending on airframe and weight). Without this, every rescue will glide/stall instead of holding altitude.

---

## How a wing rescue works (stage by stage)

When GPS Rescue triggers (failsafe or switch), the wing goes through the following stages:

### 1. Initialize
The flight controller checks that GPS, home position, and sat count are valid. If anything is wrong it goes into "do nothing" (slow controlled descent) or aborts.

### 2. Fly Home
The wing climbs to the chosen return altitude and flies toward home.
- Return altitude is determined by `gps_rescue_altitude_mode`:
  - **FIXED** – use `gps_rescue_return_alt` (metres above home).
  - **CURRENT** – current altitude + `gps_rescue_initial_climb`.
  - **MAX** (default) – highest altitude reached so far in the flight + `gps_rescue_initial_climb`.
- There is **no climb‑rate setting on wings**. The wing climbs (or descends) toward the target altitude as fast as the altitude PID and the maximum pitch angle (`gps_rescue_max_rescue_angle`) allow.
- The wing maintains altitude using **pitch** (tuned via `ap_wing_alt_p / _i / _d`).
- The wing steers toward home by banking (**roll**), tuned via `ap_wing_cog_p / _i / _d`.
- Cruise throttle comes from `hover_throttle` and is automatically adjusted for battery sag and pitch angle (see `tpa_speed_est_*` group).

### 3. Descend to Loiter (optional)
Once inside `gps_rescue_descent_dist` of home, the wing starts descending at `gps_rescue_descend_rate` until it reaches `ap_wing_loiter_alt`.
- Skipped entirely if `ap_wing_loiter_alt` or `ap_wing_loiter_seconds` is 0 — in that case the wing goes straight to "Descend to Land".

### 4. Loiter
The wing orbits over home at `ap_wing_loiter_alt` for `ap_wing_loiter_seconds`.
- Useful if you want time to walk to the landing area, or for the battery to settle before the final approach.

### 5. Descend to Land
The wing descends further at `gps_rescue_descend_rate` until it reaches `ap_wing_landing_alt` (the final approach altitude).

### 6. Approach Maneuver
The wing flies *outbound* from home until it is farther than `ap_wing_landing_approach_dist`. This sets up a straight‑in approach line.

### 7. Wait For Course
The wing turns onto the landing heading. It waits until it is aligned (within ~20°), or for up to 1 minute, then commits.

### 8. Last Course Adjustment
Final 3‑second lineup. Commits to land when course is within 3°, or when the wing is closer than 15 m to home, or after 3 seconds — whichever comes first.

### 9. Land
**Motor goes to 0** and the wing glides down on its final heading.
- Sink rate is `gps_rescue_descend_rate`.
- After 2 seconds, impact detection arms (`gps_rescue_disarm_threshold` controls how sensitive). On impact the wing disarms.

### Abort / Do Nothing
If a sanity check fails (no GPS, fly‑away, stall, low sats, etc.), the wing will either:
- **Abort** — disarm immediately, or
- **Do Nothing** — 20 seconds of slow controlled descent with impact disarm, giving you a chance to recover with the rescue switch.

The behaviour is controlled by `gps_rescue_sanity_checks`.

---

## CLI parameter reference

### Wing‑specific tuning parameters (`ap_wing_*`)

These exist only on wing builds. They are the primary tuning knobs for the wing rescue.

#### Altitude (pitch) PID — holds altitude during fly‑home, loiter and descent

| CLI name | Default | Range | Meaning |
|---|---|---|---|
| `ap_wing_alt_p` | 15 | 0–255 | How aggressively the wing pitches to correct altitude error. Too low: sluggish, drifts off altitude. Too high: porpoising (nose oscillates up/down). |
| `ap_wing_alt_i` | 15 | 0–255 | Removes long‑term altitude offset (e.g. wind, weight, trim). Too high: slow oscillation. Auto‑disabled when altitude error is greater than 15 m. |
| `ap_wing_alt_d` | 20 | 0–255 | Damps the altitude response so the wing doesn't overshoot when reaching a new altitude. |

#### Course (roll) PID — steers the wing toward home / landing line

| CLI name | Default | Range | Meaning |
|---|---|---|---|
| `ap_wing_cog_p` | 15 | 0–255 | How hard the wing banks to correct heading error. Too low: lazy turns, can't fight wind. Too high: rolls back and forth toward the line. |
| `ap_wing_cog_i` | 15 | 0–255 | Removes persistent heading bias (crosswind). Auto‑disabled if heading error is greater than 20°. |
| `ap_wing_cog_d` | 20 | 0–255 | Damps heading correction so the wing doesn't S‑turn. |

#### Axis mixing

| CLI name | Default | Range | Meaning |
|---|---|---|---|
| `ap_wing_roll_pitch_mix` | 0 | 0–255 | How much pitch compensation is applied when the wing banks. Value is a percentage (e.g. 50 = 50 %). Use this if the wing loses or gains pitch unexpectedly during banked turns. |
| `ap_wing_roll_yaw_mix` | 0 | 0–255 | How much rudder/yaw is added during banked turns (coordinated turn mix). Value is a percentage. Increase if the wing slips outward when banking. Typical starting point: 20–40. |

#### Loiter — orbit over home before landing

| CLI name | Default | Range | Meaning |
|---|---|---|---|
| `ap_wing_loiter_alt` | 60 | 0–255 (m AGL) | Altitude to orbit at, in metres above home. Set to **0** to skip the loiter phase entirely. |
| `ap_wing_loiter_seconds` | 60 | 0–1000 (s) | How long to orbit before starting the landing sequence. Set to **0** to skip. |

#### Landing approach

| CLI name | Default | Range | Meaning |
|---|---|---|---|
| `ap_wing_landing_alt` | 12 | 0–255 (m AGL) | Final approach altitude in metres above home. The wing stops actively descending below this height — only as low as your barometer/sonar accuracy allows. |
| `ap_wing_landing_approach_dist` | 130 | 0–1000 (m) | How far the wing flies away from home before turning inbound for landing. Increase for shallow gliders (longer final), decrease for steeper approaches. |

---

### Standard GPS Rescue parameters that also apply to wings

These are shared with the multirotor rescue but are still used on wings.

| CLI name | Default | Meaning |
|---|---|---|
| `gps_rescue_return_alt` | varies | Altitude (metres above home) used when `gps_rescue_altitude_mode = FIXED`. Ignored in CURRENT / MAX modes. |
| `gps_rescue_altitude_mode` | MAX | How return altitude is chosen: `FIXED` (use `gps_rescue_return_alt`), `CURRENT` (current alt + initial climb), or `MAX` (highest alt seen + initial climb). |
| `gps_rescue_initial_climb` | varies | Extra metres added on top of current/max altitude in CURRENT and MAX modes. Also defines the minimum height the wing must reach before turning toward home. |
| `gps_rescue_descend_rate` | 150 cm/s | Sink rate used during descend‑to‑loiter, descend‑to‑land and final landing glide. (Wings have no equivalent ascend‑rate setting — climbs are paced by the altitude PID.) |
| `gps_rescue_descent_dist` | varies | Distance from home at which the wing begins to descend. |
| `gps_rescue_max_rescue_angle` | 45 | Maximum bank/pitch the rescue is allowed to command, in degrees. Also effectively caps climb/dive rate. |
| `gps_rescue_min_start_dist` | varies | Minimum distance from home below which a rescue immediately disarms (you're already there). |
| `gps_rescue_min_sats` | 8 | Minimum sat count required for the rescue to operate. |
| `gps_rescue_allow_arming_without_fix` | OFF | If ON, you can arm without GPS fix (rescue will simply not be available). |
| `gps_rescue_sanity_checks` | FAILSAFE_ONLY | What to do on a sanity check failure: OFF, ON (immediate abort), or FAILSAFE_ONLY. |
| `gps_rescue_disarm_threshold` | 30 | Impact‑detection sensitivity at landing (lower = more sensitive). |

---

### Throttle‑related parameters (not in the `gps_rescue_*` group, but used by wing rescue)

The wing rescue does not have its own throttle PID. Instead, it uses your existing autopilot/TPA settings.

| CLI name | Default | Meaning |
|---|---|---|
| `hover_throttle` | 1275 | The **cruise throttle** the wing will use during rescue (PWM µs, 1100–1700). Set this to the throttle value that holds level flight on your wing at a normal cruise speed. **This is the single most important throttle setting for the wing rescue.** |
| `tpa_speed_est_max_voltage` | varies | Battery voltage at which `hover_throttle` is correct. Below this voltage, the rescue will automatically increase throttle to compensate for sag. |
| `tpa_speed_est_*` (basic or advanced group) | varies | Configures the airframe's thrust‑to‑weight ratio used by the rescue to add throttle when climbing (nose‑up) and reduce it when descending (nose‑down). |
| `min_check` | 1050 | Lower stick threshold; also used as the lower PWM bound when converting `hover_throttle` to a 0–100 % value. |

---

### Angle‑mode trim (critical for wings)

GPS Rescue commands a pitch *angle* to control altitude. That angle is fed through the same Angle‑mode controller you use in everyday flight, so if Angle mode is not trimmed correctly your wing will not fly level in rescue either — the altitude PID will spend all its effort fighting a built‑in bias.

| CLI name | Default | Range | Meaning |
|---|---|---|---|
| `angle_pitch_offset` | 0 | −128 to 127 (units = 0.1°) | Wing‑only. Trims the zero‑stick pitch target in Angle mode. Positive values lower the nose, negative values raise it. Example: a value of `−20` adds 2.0° of nose‑up at zero pitch stick. Adjust until your wing flies level in Angle mode at cruise speed with no stick input. |

**How to set it:**
1. Fly the wing in **Angle mode** at typical cruise speed (and with `hover_throttle` already set).
2. Note whether the wing climbs or dives at zero pitch stick.
3. If it dives, make `angle_pitch_offset` more negative (more nose up). If it climbs, make it more positive (more nose down).
4. Iterate in steps of 5–10 (0.5–1.0°) until level flight is hands‑off.

Without this set correctly, `ap_wing_alt_i` will eventually compensate — but very slowly, and only within its ±15 m error window — so altitude tracking during rescue will be sloppy.

---

## Suggested tuning order

1. **Set `hover_throttle`.** Fly your wing manually at a comfortable cruise and note the throttle value that holds level flight. Set `hover_throttle` to that value. Without this, nothing else will work right.
2. **Trim Angle mode with `angle_pitch_offset`.** Fly in Angle mode at cruise and adjust `angle_pitch_offset` until the wing holds level with no stick input. GPS Rescue runs through the Angle controller, so this trim is essential.
3. **Configure TPA speed / TWR.** Make sure the `tpa_speed_est_*` group reflects your airframe so the rescue can compensate throttle for climb/descent.
4. **Tune altitude hold.** Trigger a rescue from a known altitude. Adjust `ap_wing_alt_p` until the wing tracks altitude firmly without porpoising. Add `ap_wing_alt_d` to remove overshoot. Add just enough `ap_wing_alt_i` to kill steady offset.
5. **Tune heading hold.** Same order with `ap_wing_cog_p / _d / _i`. If the wing weaves toward home, reduce P or add D. If it can't quite reach the line in wind, add I.
6. **Tune mixers.** If the wing slips outward during turns, add a little `ap_wing_roll_yaw_mix` (try 20–40). If pitch behaves oddly during banked turns, use `ap_wing_roll_pitch_mix`.
7. **Set up landing geometry.** Adjust `ap_wing_landing_approach_dist` to give your wing room for its glide angle. Set `ap_wing_landing_alt` only as low as your altitude sensors are reliable.
8. **Decide on loiter.** If you want the wing to orbit before landing, set `ap_wing_loiter_alt` and `ap_wing_loiter_seconds`. Otherwise leave both at 0 to skip loiter.

---

## Quick safety notes

- **Test high.** First rescues should always be triggered with plenty of altitude — never close to the ground.
- **Motor is cut on landing.** From the "Last Course Adjustment" stage onward, throttle is forced to 0 and the wing glides. Plan for that glide distance.
- **Wind matters.** A strong tailwind on final can dramatically extend the glide. Use a longer `ap_wing_landing_approach_dist` or aim into wind.
- **No compass needed.** The wing rescue steers using GPS course over ground, not the magnetometer. `gps_rescue_use_mag` has no effect on a wing.

---

*Applies to Betaflight branch `NEW_WING_4.6_GPS` (Wing RTH).*
