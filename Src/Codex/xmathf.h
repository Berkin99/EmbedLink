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

#ifndef XMATHF_H_
#define XMATHF_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <sysdefs.h>
#include <systime.h>
#include <xmath_types.h>
#include <xmath3d.h>

typedef struct{
    f32 value;
    f32 stdDev;
    u32 timestampMs;
}xf32_t;

typedef struct{
    f64 value;
    f64 stdDev;
    u32 timestampMs;
}xf64_t;

typedef struct{
    union{
        struct{
            f32 x;
            f32 y;
        };
        f32 axis[2];
        v2f32_t v;
    };
    v2f32_t stdDev;
    u32     timestampMs;
}xv2f32_t;

typedef struct{
    union{
        struct{
            f64 x;
            f64 y;
        };
        f64 axis[2];
        v2f64_t v;
    };
    v2f64_t stdDev;
    u32     timestampMs;
}xv2f64_t;

typedef struct{
    union{
        struct{
            f32 x;
            f32 y;
            f32 z;
        };
        f32 axis[3];
        v3f32_t v;
    };
    v3f32_t stdDev;
    u32     timestampMs;
}xv3f32_t;

typedef struct{
    union{
        struct{
            f64 x;
            f64 y;
            f64 z;
        };
        f64 axis[3];
        v3f64_t v;
    };
    v3f64_t stdDev;
    u32     timestampMs;
}xv3f64_t;

typedef xv3f32_t xvec_t;

/* Create a new scalar uncertainty value */
static inline xf32_t xfnew(f32 value, f32 stdDev, u32 timestampMs) {
    return (xf32_t){value, stdDev, timestampMs};
}

/* Create a new vector uncertainty value with different stdDevs */
static inline xvec_t xvnew(vec_t v, vec_t stdDev, u32 timestampMs) {
    return (xvec_t){
        .x = v.x,
        .y = v.y,
        .z = v.z,
        .stdDev = stdDev,
        .timestampMs = timestampMs
    };
}

/* Create a new scalar xvalue */
static inline xf32_t xfzero(void) {
    return (xf32_t){0.0f, 0.0f, 0.0f};
}

/* Create a new zero xvector */
static inline xvec_t xvzero(void) {
    return (xvec_t){
        .v = vzero(),
        .stdDev = vzero(),
        .timestampMs = 0U
    };
}

/* Create a new scalar value with zero uncertainty */
static inline xf32_t xfcert(f32 value, u32 timestampMs) {
    return xfnew(value, 0.0f, timestampMs);
}

/* Create a new vector value with zero uncertainty */
static inline xvec_t xvcert(vec_t v, u32 timestampMs) {
    return xvnew(v, vzero(), timestampMs);
}

/* Update timestamp in milliseconds */
static inline void xfupdate(xf32_t* xf) {
    xf->timestampMs = millis();
}

/* Update timestamp in milliseconds */
static inline void xvupdate(xvec_t* xv) {
    xv->timestampMs = millis();
}

/* Float Timeout */
static inline bool xftime(xf32_t* xf, u32 timeout) {
    return ((xf->timestampMs + timeout) > millis());
}

/* Vector Timeout */
static inline bool xvtime(xvec_t* xv, u32 timeout) {
    return ((xv->timestampMs + timeout) > millis());
}

/* Combine two scalar measurements using weighted average */
static inline xf32_t xfcomb(xf32_t a, xf32_t b) {
    if (a.stdDev <= 0.0f) return a;
    if (b.stdDev <= 0.0f) return b;
    
    f32 wa = 1.0f / (a.stdDev * a.stdDev);
    f32 wb = 1.0f / (b.stdDev * b.stdDev);
    f32 value = (a.value * wa + b.value * wb) / (wa + wb);
    f32 stdDev = sqrtf(1.0f / (wa + wb));
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return xfnew(value, stdDev, timestampMs);
}

/* Combine two vector measurements using weighted average */
static inline xvec_t xvcomb(xvec_t a, xvec_t b) {
    xvec_t result;
    
    vec_t wa = veltrecip(vec2(a.stdDev));
    vec_t wb = veltrecip(vec2(b.stdDev));
    vec_t w_sum = vadd(wa, wb);
    
    result.v = veltdiv(vadd(veltmul(a.v, wa), veltmul(b.v, wb)), w_sum);
    result.stdDev = vsqrt(veltrecip(w_sum));
    result.timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return result;
}

/* Add two uncertain scalars - propagate uncertainty */
static inline xf32_t xfadd(xf32_t a, xf32_t b) {
    f32 value = a.value + b.value;
    f32 stdDev = sqrtf(a.stdDev * a.stdDev + b.stdDev * b.stdDev);
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return xfnew(value, stdDev, timestampMs);
}

/* Subtract two uncertain scalars - propagate uncertainty */
static inline xf32_t xfsub(xf32_t a, xf32_t b) {
    f32 value = a.value - b.value;
    f32 stdDev = sqrtf(a.stdDev * a.stdDev + b.stdDev * b.stdDev);
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return xfnew(value, stdDev, timestampMs);
}

/* Multiply two uncertain scalars - propagate uncertainty */
static inline xf32_t xfmul(xf32_t a, xf32_t b) {
    f32 value = a.value * b.value;
    /* Relative uncertainties add in quadrature for multiplication */
    f32 rel_a = (a.value != 0.0f) ? (a.stdDev / fabsf(a.value)) : 0.0f;
    f32 rel_b = (b.value != 0.0f) ? (b.stdDev / fabsf(b.value)) : 0.0f;
    f32 rel = sqrtf(rel_a * rel_a + rel_b * rel_b);
    f32 stdDev = fabsf(value) * rel;
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return xfnew(value, stdDev, timestampMs);
}

/* Divide two uncertain scalars - propagate uncertainty */
static inline xf32_t xfdiv(xf32_t a, xf32_t b) {
    f32 value = a.value / b.value;
    /* Relative uncertainties add in quadrature for division */
    f32 rel_a = (a.value != 0.0f) ? (a.stdDev / fabsf(a.value)) : 0.0f;
    f32 rel_b = (b.value != 0.0f) ? (b.stdDev / fabsf(b.value)) : 0.0f;
    f32 rel = sqrtf(rel_a * rel_a + rel_b * rel_b);
    f32 stdDev = fabsf(value) * rel;
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return xfnew(value, stdDev, timestampMs);
}

/* Multiply uncertain scalar by a constant */
static inline xf32_t xfscl(xf32_t a, f32 s) {
    return xfnew(a.value * s, a.stdDev * fabsf(s), a.timestampMs);
}

/* Add two uncertain vectors - propagate uncertainty */
static inline xvec_t xvadd(xvec_t a, xvec_t b) {
    xvec_t result;
    
    result.v = vadd(a.v, b.v);
    result.stdDev = vsqrt(vadd(vec2(a.stdDev), vec2(b.stdDev)));
    result.timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return result;
}

/* Subtract two uncertain vectors - propagate uncertainty */
static inline xvec_t xvsub(xvec_t a, xvec_t b) {
    xvec_t result;
    
    result.v = vsub(a.v, b.v);
    result.stdDev = vsqrt(vadd(vec2(a.stdDev), vec2(b.stdDev)));
    result.timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    
    return result;
}

/* Scale uncertain vector by a constant */
static inline xvec_t xvscl(xvec_t a, f32 s) {
    xvec_t result;
    
    result.v = vscl(a.v, s);
    result.stdDev = vscl(a.stdDev, fabsf(s));
    result.timestampMs = a.timestampMs;
    
    return result;
}

/* Calculate magnitude of uncertain vector */
static inline xf32_t xvmag(xvec_t v) {
    f32 mag = vmag(v.v);
    
    /* Propagate uncertainty for magnitude calculation */
    vec_t norm = veltdiv(v.v, vrepeat(mag));
    f32 stdDev = sqrtf(vdot(veltmul(norm, v.stdDev), veltmul(norm, v.stdDev)));
    
    return xfnew(mag, stdDev, v.timestampMs);
}

/* Calculate dot product of two uncertain vectors */
static inline xf32_t xvdot(xvec_t a, xvec_t b) {
    f32 dot = vdot(a.v, b.v);
    
    /* Calculate uncertainty of dot product */
    f32 var_a = vdot(vec2(b.v), vec2(a.stdDev));
    f32 var_b = vdot(vec2(a.v), vec2(b.stdDev));
    f32 stdDev = sqrtf(var_a + var_b);
    
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return xfnew(dot, stdDev, timestampMs);
}

/* Calculate distance between two uncertain vectors */
static inline xf32_t xvdist(xvec_t a, xvec_t b) {
    return xvmag(xvsub(a, b));
}

/* Calculate gaussian probability density function */
static inline f32 xpdf(f32 x, f32 mean, f32 stdDev) {
    f32 z = (x - mean) / stdDev;
    return expf(-0.5f * z * z) / (stdDev * sqrtf(2.0f * M_PI_F32));
}

/* Calculate cumulative distribution function */
static inline f32 xcdf(f32 x, f32 mean, f32 stdDev) {
    f32 z = (x - mean) / stdDev;
    return 0.5f * (1.0f + erff(z / sqrtf(2.0f)));
}

/* Calculate probability of a value being within range */
static inline f32 xprange(f32 a, f32 b, f32 mean, f32 stdDev) {
    return xcdf(b, mean, stdDev) - xcdf(a, mean, stdDev);
}

/* Calculate probability of a value being > threshold */
static inline f32 xpgt(xf32_t value, f32 threshold) {
    return 1.0f - xcdf(threshold, value.value, value.stdDev);
}

/* Calculate probability of a value being < threshold */
static inline f32 xplt(xf32_t value, f32 threshold) {
    return xcdf(threshold, value.value, value.stdDev);
}

/* Calculate z-score for a given value */
static inline f32 xzscore(f32 x, f32 mean, f32 stdDev) {
    return (x - mean) / stdDev;
}

/* Calculate confidence interval for 95% */
static inline f32 xci95(f32 stdDev) {
    return 1.96f * stdDev;
}

/* Calculate confidence interval for 99% */
static inline f32 xci99(f32 stdDev) {
    return 2.576f * stdDev;
}

/* Get confidence interval for an uncertain value */
static inline void xci(xf32_t value, f32 confidence, f32 *lower, f32 *upper) {
    f32 z;
    if (confidence >= 0.99f) z = 2.576f;
    else if (confidence >= 0.95f) z = 1.96f;
    else if (confidence >= 0.90f) z = 1.645f;
    else if (confidence >= 0.80f) z = 1.282f;
    else z = 1.0f;  /* Default to 68% confidence */
    
    f32 interval = z * value.stdDev;
    *lower = value.value - interval;
    *upper = value.value + interval;
}

/* Kalman filter update for 1D (scalar) case */
static inline xf32_t xfkalman(xf32_t prior, xf32_t meas) {
    f32 kg = prior.stdDev * prior.stdDev / 
            (prior.stdDev * prior.stdDev + meas.stdDev * meas.stdDev);
    
    f32 value = prior.value + kg * (meas.value - prior.value);
    f32 stdDev = sqrtf((1.0f - kg) * prior.stdDev * prior.stdDev);
    
    return xfnew(value, stdDev, meas.timestampMs);
}

/* Simple prediction based on velocity and time */
static inline xvec_t xvpredict(xvec_t pos, xvec_t vel, f32 dt) {
    xvec_t pred;
    
    /* Position = position + velocity * dt */
    pred.v = vadd(pos.v, vscl(vel.v, dt));
    
    /* Uncertainty grows with prediction */
    vec_t dt_vec = vrepeat(dt);
    pred.stdDev = vsqrt(vadd(vec2(pos.stdDev), veltmul(vec2(vel.stdDev), vec2(dt_vec))));
    pred.timestampMs = pos.timestampMs + (u32)(dt * 1000.0f);
    
    return pred;
}

/* Calculate entropy for a gaussian distribution */
static inline f32 xentropy(f32 stdDev) {
    return 0.5f * logf(2.0f * M_PI_F32 * M_E_F32 * stdDev * stdDev);
}

/* Add random gaussian noise to a value */
static inline f32 xnoise(f32 value, f32 stdDev) {
    /* Box-Muller transform for Gaussian random numbers */
    f32 u1 = (f32)rand() / (f32)RAND_MAX;
    f32 u2 = (f32)rand() / (f32)RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;  /* Avoid log(0) */
    
    f32 z = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI_F32 * u2);
    return value + stdDev * z;
}

/* Normalize uncertain vector - propagate uncertainty */
f32 xvnorm(xvec_t v, xvec_t *result);

/* Calculate cross product of uncertain vectors */
xvec_t xvcross(xvec_t a, xvec_t b);

/* Calculate Mahalanobis distance */
f32 xmahal(vec_t point, vec_t mean, vec_t stdDev);

/* Kalman filter update for 3D vector case */
xvec_t xvkalman(xvec_t prior, xvec_t meas);

/* Calculate vector entropy */
f32 xventropy(xvec_t vec);

/* Calculate weighted mixture of uncertain values */
xf32_t xfmix(xf32_t a, xf32_t b, f32 w_a);

/* Calculate KL divergence between 1D Gaussians */
f32 xkldiv(f32 mean1, f32 stdDev1, f32 mean2, f32 stdDev2);

/* Calculate JS divergence between 1D Gaussians */
f32 xjsdiv(f32 mean1, f32 stdDev1, f32 mean2, f32 stdDev2);

/* Time decay for uncertain values */
xf32_t xfdecay(xf32_t value, u32 current_time_ms, f32 half_life_ms);

/* Weighted average of multiple values */
xf32_t xfwavg(xf32_t *values, f32 *weights, size_t count);

/* Monte Carlo simulation for uncertain vector */
vec_t xvsample(xvec_t vec);

/* Calculate probability of vector in sphere */
f32 xpinsphere(xvec_t center, xvec_t point, f32 radius);

/* Sample from multivariate normal */
vec_t xmvnorm(vec_t mean, vec_t stdDev);

/* Compute correlation coefficient */
f32 xcorr(f32 *x_samples, f32 *y_samples, size_t count);

/* Bayesian update of prior given new evidence */
f32 xbayes(f32 prior, f32 likelihood_if_true, f32 likelihood_if_false);

#endif /* XMATHF_H_ */