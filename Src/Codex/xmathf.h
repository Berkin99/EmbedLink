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
#include <math.h>
#include <sysdefs.h>
#include <systime.h>
#include <xmath_types.h>
#include <xmath3d.h>

/* Uncertain scalar v (float, standard deviation, timestamp) */
typedef struct {
    f32 v;
    f32 stdDev;
    u32 timestampMs;
} xf32_t;

/* Uncertain scalar v (double, standard deviation, timestamp) */
typedef struct {
    f64 v;
    f64 stdDev;
    u32 timestampMs;
} xf64_t;

/* 2D uncertain vector (float) with standard deviation and timestamp */
typedef struct {
    union {
        struct { f32 x, y; };
        f32 axis[2];
        v2f32_t v;
    };
    v2f32_t stdDev;
    u32     timestampMs;
} xv2f32_t;

/* 2D uncertain vector (double) with standard deviation and timestamp */
typedef struct {
    union {
        struct { f64 x, y; };
        f64 axis[2];
        v2f64_t v;
    };
    v2f64_t stdDev;
    u32     timestampMs;
} xv2f64_t;

/* 3D uncertain vector (float) with standard deviation and timestamp */
typedef struct {
    union {
        struct { f32 x, y, z; };
        f32 axis[3];
        v3f32_t v;
    };
    v3f32_t stdDev;
    u32     timestampMs;
} xv3f32_t;

/* 3D uncertain vector (double) with standard deviation and timestamp */
typedef struct {
    union {
        struct { f64 x, y, z; };
        f64 axis[3];
        v3f64_t v;
    };
    v3f64_t stdDev;
    u32     timestampMs;
} xv3f64_t;

/* Default uncertain vector type (3D, float) */
typedef xv3f32_t xvec_t;

/* Create a new uncertain scalar v */
static inline xf32_t xfnew(f32 v, f32 stdDev, u32 timestampMs) {
    return (xf32_t){ v, stdDev, timestampMs };
}

/* Create a new uncertain vector with std deviation and timestamp */
static inline xvec_t xvnew(vec_t v, vec_t stdDev, u32 timestampMs) {
    xvec_t xv;
    xv.v = v;
    xv.stdDev = stdDev;
    xv.timestampMs = timestampMs;
    return xv;
}

/* Return a zero-vd uncertain scalar */
static inline xf32_t xfzero(void) {
    return (xf32_t){ 0.0f, 0.0f, 0U };
}

/* Return a zero-vd uncertain vector */
static inline xvec_t xvzero(void) {
    return xvnew(vzero(), vzero(), 0U);
}

/* Create an uncertain scalar v with zero uncertainty */
static inline xf32_t xfcert(f32 v, u32 timestampMs) {
    return xfnew(v, 0.0f, timestampMs);
}

/* Create an uncertain vector with zero uncertainty */
static inline xvec_t xvcert(vec_t v, u32 timestampMs) {
    return xvnew(v, vzero(), timestampMs);
}

/* Update the timestamp of an uncertain scalar to the current time */
static inline void xfupdate(xf32_t* xf) {
    xf->timestampMs = millis();
}

/* Update the timestamp of an uncertain vector to the current time */
static inline void xvupdate(xvec_t* xv) {
    xv->timestampMs = millis();
}

/* Test if an uncertain scalar is within a timeout period */
static inline bool xftime(xf32_t* xf, u32 timeout) {
    return ((xf->timestampMs + timeout) > millis());
}

/* Timeout : 0  | Time ok : 1 */
static inline bool xvtime(xvec_t* xv, u32 timeout) {
    return ((xv->timestampMs + timeout) > millis());
}

/* Combine two uncertain scalars using weighted mean */
static inline xf32_t xfcomb(xf32_t a, xf32_t b) {
    if (a.stdDev <= 0.0f) return a;
    if (b.stdDev <= 0.0f) return b;

    f32 wa = 1.0f / (a.stdDev * a.stdDev);
    f32 wb = 1.0f / (b.stdDev * b.stdDev);
    f32 v = (a.v * wa + b.v * wb) / (wa + wb);
    f32 stdDev = sqrtf(1.0f / (wa + wb));
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;

    return xfnew(v, stdDev, timestampMs);
}

/* Combine two uncertain vectors using weighted mean */
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

/* Add two uncertain scalars and propagate uncertainty */
static inline xf32_t xfadd(xf32_t a, xf32_t b) {
    f32 v = a.v + b.v;
    f32 stdDev = sqrtf(a.stdDev * a.stdDev + b.stdDev * b.stdDev);
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return xfnew(v, stdDev, timestampMs);
}

/* Subtract two uncertain scalars and propagate uncertainty */
static inline xf32_t xfsub(xf32_t a, xf32_t b) {
    f32 v = a.v - b.v;
    f32 stdDev = sqrtf(a.stdDev * a.stdDev + b.stdDev * b.stdDev);
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return xfnew(v, stdDev, timestampMs);
}

/* Multiply two uncertain scalars and propagate uncertainty */
static inline xf32_t xfmul(xf32_t a, xf32_t b) {
    f32 v = a.v * b.v;
    f32 rel_a = (a.v != 0.0f) ? (a.stdDev / fabsf(a.v)) : 0.0f;
    f32 rel_b = (b.v != 0.0f) ? (b.stdDev / fabsf(b.v)) : 0.0f;
    f32 rel = sqrtf(rel_a * rel_a + rel_b * rel_b);
    f32 stdDev = fabsf(v) * rel;
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return xfnew(v, stdDev, timestampMs);
}

/* Divide two uncertain scalars and propagate uncertainty */
static inline xf32_t xfdiv(xf32_t a, xf32_t b) {
    f32 v = a.v / b.v;
    f32 rel_a = (a.v != 0.0f) ? (a.stdDev / fabsf(a.v)) : 0.0f;
    f32 rel_b = (b.v != 0.0f) ? (b.stdDev / fabsf(b.v)) : 0.0f;
    f32 rel = sqrtf(rel_a * rel_a + rel_b * rel_b);
    f32 stdDev = fabsf(v) * rel;
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return xfnew(v, stdDev, timestampMs);
}

/* Scale an uncertain scalar by a constant */
static inline xf32_t xfscl(xf32_t a, f32 s) {
    return xfnew(a.v * s, a.stdDev * fabsf(s), a.timestampMs);
}

/* Add two uncertain vectors and propagate uncertainty */
static inline xvec_t xvadd(xvec_t a, xvec_t b) {
    xvec_t result;
    result.v = vadd(a.v, b.v);
    result.stdDev = vsqrt(vadd(vec2(a.stdDev), vec2(b.stdDev)));
    result.timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return result;
}

/* Subtract two uncertain vectors and propagate uncertainty */
static inline xvec_t xvsub(xvec_t a, xvec_t b) {
    xvec_t result;
    result.v = vsub(a.v, b.v);
    result.stdDev = vsqrt(vadd(vec2(a.stdDev), vec2(b.stdDev)));
    result.timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return result;
}

/* Scale an uncertain vector by a constant */
static inline xvec_t xvscl(xvec_t a, f32 s) {
    xvec_t result;
    result.v = vscl(a.v, s);
    result.stdDev = vscl(a.stdDev, fabsf(s));
    result.timestampMs = a.timestampMs;
    return result;
}

/* Calculate the magnitude of an uncertain vector and propagate uncertainty */
static inline xf32_t xvmag(xvec_t v) {
    f32 mag = vmag(v.v);
    vec_t norm = veltdiv(v.v, vrepeat(mag));
    f32 stdDev = sqrtf(vdot(veltmul(norm, v.stdDev), veltmul(norm, v.stdDev)));
    return xfnew(mag, stdDev, v.timestampMs);
}

/* Dot product of two uncertain vectors with propagated uncertainty */
static inline xf32_t xvdot(xvec_t a, xvec_t b) {
    f32 dot = vdot(a.v, b.v);
    f32 var_a = vdot(vec2(b.v), vec2(a.stdDev));
    f32 var_b = vdot(vec2(a.v), vec2(b.stdDev));
    f32 stdDev = sqrtf(var_a + var_b);
    u32 timestampMs = (a.timestampMs > b.timestampMs) ? a.timestampMs : b.timestampMs;
    return xfnew(dot, stdDev, timestampMs);
}

/* Distance between two uncertain vectors */
static inline xf32_t xvdist(xvec_t a, xvec_t b) {
    return xvmag(xvsub(a, b));
}

/* Probability density function of a normal distribution */
static inline f32 xpdf(f32 x, f32 mean, f32 stdDev) {
    f32 z = (x - mean) / stdDev;
    return expf(-0.5f * z * z) / (stdDev * sqrtf(2.0f * M_PI_F32));
}

/* Cumulative distribution function of a normal distribution */
static inline f32 xcdf(f32 x, f32 mean, f32 stdDev) {
    f32 z = (x - mean) / stdDev;
    return 0.5f * (1.0f + erff(z / sqrtf(2.0f)));
}

/* Probability that a v is within a range under a normal distribution */
static inline f32 xprange(f32 a, f32 b, f32 mean, f32 stdDev) {
    return xcdf(b, mean, stdDev) - xcdf(a, mean, stdDev);
}

/* Probability that an uncertain scalar is greater than a threshold */
static inline f32 xpgt(xf32_t v, f32 threshold) {
    return 1.0f - xcdf(threshold, v.v, v.stdDev);
}

/* Probability that an uncertain scalar is less than a threshold */
static inline f32 xplt(xf32_t v, f32 threshold) {
    return xcdf(threshold, v.v, v.stdDev);
}

/* Calculate the z-score of a v in a normal distribution */
static inline f32 xzscore(f32 x, f32 mean, f32 stdDev) {
    return (x - mean) / stdDev;
}

/* Calculate 95% confidence interval width */
static inline f32 xci95(f32 stdDev) {
    return 1.96f * stdDev;
}

/* Calculate 99% confidence interval width */
static inline f32 xci99(f32 stdDev) {
    return 2.576f * stdDev;
}

/* Calculate symmetric confidence interval around an uncertain v */
static inline void xci(xf32_t v, f32 confidence, f32 *lower, f32 *upper) {
    f32 z;
    if (confidence >= 0.99f) z = 2.576f;
    else if (confidence >= 0.95f) z = 1.96f;
    else if (confidence >= 0.90f) z = 1.645f;
    else if (confidence >= 0.80f) z = 1.282f;
    else z = 1.0f;

    f32 interval = z * v.stdDev;
    *lower = v.v - interval;
    *upper = v.v + interval;
}

/* Kalman filter update for a 1D uncertain scalar */
static inline xf32_t xfkalman(xf32_t prior, xf32_t meas) {
    f32 kg = prior.stdDev * prior.stdDev /
            (prior.stdDev * prior.stdDev + meas.stdDev * meas.stdDev);
    f32 v = prior.v + kg * (meas.v - prior.v);
    f32 stdDev = sqrtf((1.0f - kg) * prior.stdDev * prior.stdDev);
    return xfnew(v, stdDev, meas.timestampMs);
}

/* Predict the new state of an uncertain vector with velocity over dt */
static inline xvec_t xvpredict(xvec_t pos, xvec_t vel, f32 dt) {
    xvec_t pred;
    pred.v = vadd(pos.v, vscl(vel.v, dt));
    vec_t dt_vec = vrepeat(dt);
    pred.stdDev = vsqrt(vadd(vec2(pos.stdDev), veltmul(vec2(vel.stdDev), vec2(dt_vec))));
    pred.timestampMs = pos.timestampMs + (u32)(dt * 1000.0f);
    return pred;
}

/* Entropy of a normal distribution */
static inline f32 xentropy(f32 stdDev) {
    return 0.5f * logf(2.0f * M_PI_F32 * M_E_F32 * stdDev * stdDev);
}

/* Add random Gaussian noise to a v */
static inline f32 xnoise(f32 v, f32 stdDev) {
    f32 u1 = (f32)rand() / (f32)RAND_MAX;
    f32 u2 = (f32)rand() / (f32)RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    f32 z = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI_F32 * u2);
    return v + stdDev * z;
}

/* Calculate vector norm and propagate uncertainty (implementation required) */
f32 xvnorm(xvec_t v, xvec_t *result);

/* Compute cross product of two uncertain vectors (implementation required) */
xvec_t xvcross(xvec_t a, xvec_t b);

/* Compute Mahalanobis distance for a vector (implementation required) */
f32 xmahal(vec_t point, vec_t mean, vec_t stdDev);

/* Kalman filter update for 3D uncertain vector (implementation required) */
xvec_t xvkalman(xvec_t prior, xvec_t meas);

/* Calculate entropy of a 3D uncertain vector (implementation required) */
f32 xventropy(xvec_t vec);

/* Weighted mixture of two uncertain scalar vs (implementation required) */
xf32_t xfmix(xf32_t a, xf32_t b, f32 w_a);

/* Kullback-Leibler divergence between two 1D Gaussians (implementation required) */
f32 xkldiv(f32 mean1, f32 stdDev1, f32 mean2, f32 stdDev2);

/* Jensen-Shannon divergence between two 1D Gaussians (implementation required) */
f32 xjsdiv(f32 mean1, f32 stdDev1, f32 mean2, f32 stdDev2);

/* Apply time-based exponential decay to uncertain scalar (implementation required) */
xf32_t xfdecay(xf32_t v, u32 current_time_ms, f32 half_life_ms);

/* Weighted average of uncertain scalar vs (implementation required) */
xf32_t xfwavg(xf32_t *vs, f32 *weights, size_t count);

/* Draw a random sample from an uncertain vector (implementation required) */
vec_t xvsample(xvec_t vec);

/* Probability that an uncertain point lies within a sphere (implementation required) */
f32 xpinsphere(xvec_t center, xvec_t point, f32 radius);

/* Sample from multivariate normal distribution (implementation required) */
vec_t xmvnorm(vec_t mean, vec_t stdDev);

/* Correlation coefficient between two sample arrays (implementation required) */
f32 xcorr(f32 *x_samples, f32 *y_samples, size_t count);

/* Bayesian update for scalar probabilities (implementation required) */
f32 xbayes(f32 prior, f32 likelihood_if_true, f32 likelihood_if_false);

#endif /* XMATHF_H_ */
