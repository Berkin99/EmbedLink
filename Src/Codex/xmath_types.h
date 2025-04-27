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

#ifndef XMATH_TYPES_H_
#define XMATH_TYPES_H_

#include <stdint.h>

#define M_PI_F32   (3.14159265358979323846f)
#define M_PI_F64   (3.14159265358979323846)
#define M_E_F32    (2.7182818284590452354f)
#define M_E_F64    (2.7182818284590452354)
#define RAD2DEG    (180.0 / M_PI_F64)
#define DEG2RAD    (M_PI_F64 / 180.0)

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t   i8;
typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;
typedef float    f32;
typedef double   f64;

/* 2. Vector2 Types */
/* 2.1. Vector2 Integer Types */
typedef union {
    struct {
        i8 x;
        i8 y;
    };
    i8 axis[2];
}v2i8_t;

typedef union {
    struct {
        i16 x;
        i16 y;
    };
    i16 axis[2];
}v2i16_t;

typedef union {
    struct {
        i32 x;
        i32 y;
    };
    i32 axis[2];
}v2i32_t;

typedef union {
    struct {
        i64 x;
        i64 y;
    };
    i64 axis[2];
}v2i64_t;

/* 2.2. Vector2 Floating Types */
typedef union {
    struct {
        f32 x;
        f32 y;
    };
    f32 axis[2];
}v2f32_t;

typedef union {
    struct {
        f64 x;
        f64 y;
    };
    f64 axis[2];
}v2f64_t;

/* 3. Vector3 Types */
/* 3.1. Vector3 Integer Types */
typedef union {
    struct {
        i8 x;
        i8 y;
        i8 z;
    };
    i8 axis[3];
}v3i8_t;

typedef union {
    struct {
        i16 x;
        i16 y;
        i16 z;
    };
    i16 axis[3];
}v3i16_t;

typedef union {
    struct {
        i32 x;
        i32 y;
        i32 z;
    };
    i32 axis[3];
}v3i32_t;

typedef union {
    struct {
        i64 x;
        i64 y;
        i64 z;
    };
    i64 axis[3];
}v3i64_t;

/* 3.2. Vector3 Float Types */
typedef union {
    struct {
        f32 x;
        f32 y;
        f32 z;
    };
    f32 axis[3];
}v3f32_t;

typedef union {
    struct {
        f64 x;
        f64 y;
        f64 z;
    };
    f64 axis[3];
}v3f64_t;

#endif