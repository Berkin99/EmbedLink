/*
 *       ______          __             ____    _       __
 *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
 *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
 *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
 *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
 *
 *  EmbedLink Firmware
 *  Copyright (c) 2024 Yeniay RD, All rights reserved.
 *  _________________________________________________________
 *
 *  EmbedLink Firmware is free software: you can redistribute
 *  it and/or  modify it under  the  terms of the  GNU Lesser
 *  General Public License as  published by the Free Software
 *  Foundation,  either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  EmbedLink  Firmware is  distributed  in the  hope that it
 *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with EmbedLink Firmware. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#ifndef XMATH_H_
#define XMATH_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "xmath_types.h"

/* Clamp a f32 value between a min and max value */
static inline f32 clampf32(f32 val, f32 min, f32 max) {
	return fminf(max, fmaxf(min, val));
}

static inline f64 clampf64(f64 val, f64 min, f64 max) {
	return fmin(max, fmax(min, val));
}

/* Convert degrees to radians */
static inline f32 deg2rad(f32 degrees) { return (f32)DEG2RAD * degrees; }

/* Convert radians to degrees */
static inline f32 rad2deg(f32 radians) { return (f32)RAD2DEG * radians; }

/* Normalize radians to range [-pi, pi] */
static inline f32 radnf32(f32 radians){
    f32 signed_pi = copysignf(M_PI_F32, radians);
    radians = fmodf(radians + signed_pi, 2 * M_PI_F32) - signed_pi;
    return radians;
}

/* Compare two floats for approximate equality with ulps threshold */
static inline bool closeulpsf32(f32 a, f32 b, i32 ulps) {
    if ((a < 0.0f) != (b < 0.0f)) {
        if (a == b) {
            return true;
        }
        return false;
    }
    i32 ia = *((i32 *)&a);
    i32 ib = *((i32 *)&b);
    return fabsf(ia - ib) <= ulps;
}

static inline f32 deadbandf32(f32 val, f32 threshold){
    if (fabsf(val) < threshold) val = 0;
    else if (val > 0) val -= threshold;
    else if (val < 0) val += threshold;
    return val;
}

static inline float wsumf32(f32 w1, f32 val1, f32 w2, f32 val2){
    return (w1 * val1 + w2 * val2);
}

static inline float wavgf32(f32 w1, f32 val1, f32 w2, f32 val2){
    return wsumf32(w1, val1, w2, val2) / (w1 + w2);
}

static inline f32 cycdiff32(f32 angle1, f32 angle2){
    f32 diff = angle1 - angle2;
    if (diff < -M_PI_F32) return (diff + (2 * M_PI_F32));
    if (diff >  M_PI_F32) return (diff - (2 * M_PI_F32));
    return diff;
}

static inline f32 meanf32(f32 pmean, f32 new, i32 i) {
    return (pmean * (f32)i / (i + 1)) + (new * 1.0f / (i + 1));
}

static inline f64 meanf64(f64 pmean, f64 new, i32 i) {
    return (pmean * (f64)i / (i + 1)) + (new * 1.0 / (i + 1));
}

#endif
