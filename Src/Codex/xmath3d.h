#ifndef XMATH3D_H_
#define XMATH3D_H_

#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <xmath_types.h>

typedef v3f32_t vec_t;

/* Construct a vector from 3 floats */
static inline vec_t vnew(f32 x, f32 y, f32 z) {
    return (vec_t){x, y, z};
}

/* Add two vectors */
static inline vec_t vadd(vec_t a, vec_t b) {
    return vnew(a.x + b.x, a.y + b.y, a.z + b.z);
}

/* Subtract two vectors */
static inline vec_t vsub(vec_t a, vec_t b) {
    return vadd(a, vneg(b));
}

/* Multiply a vector by a scalar */
static inline vec_t vscl(vec_t v, f32 s) {
    return vnew(s * v.x , s * v.y, s * v.z);
}

/* Divide a vector by a scalar */
static inline vec_t vdiv(vec_t v, f32 s) {
    return vscl(v, 1.0f/s);
}

/* Negate a vector */
static inline vec_t vneg(vec_t v) {
    return vnew(-v.x, -v.y, -v.z);
}

/* Dot product of two vectors */
static inline f32 vdot(vec_t a, vec_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* Construct a vector with the same value repeated */
static inline vec_t vrepeat(f32 x) {
    return vnew(x, x, x);
}

/* Construct a zero vector */
static inline vec_t vzero(void) {
    return vrepeat(0.0f);
}

/* Construct the i'th basis vector */
static inline vec_t vbasis(int i) {
    f32 a[3] = {0.0f, 0.0f, 0.0f};
    a[i] = 1.0f;
    return vnew(a[0], a[1], a[2]);
}

/* Element-wise vector multiply */
static inline vec_t veltmul(vec_t a, vec_t b) {
    return vnew(a.x * b.x, a.y * b.y, a.z * b.z);
}

/* Element-wise vector divide */
static inline vec_t veltdiv(vec_t a, vec_t b) {
    return vnew(a.x / b.x, a.y / b.y, a.z / b.z);
}

/* Element-wise vector reciprocal */
static inline vec_t veltrecip(vec_t a) {
    return vnew(1.0f / a.x, 1.0f / a.y, 1.0f / a.z);
}

/* Vector magnitude squared */
static inline f32 vmag2(vec_t v) {
    return vdot(v, v);
}

/* Vector magnitude */
static inline f32 vmag(vec_t v) {
    return sqrtf(vmag2(v));
}

/* Euclidean distance squared between two vectors */
static inline f32 vdist2(vec_t a, vec_t b) {
    return vmag2(vsub(a, b));
}

/* Euclidean distance between two vectors */
static inline f32 vdist(vec_t a, vec_t b) {
    return sqrtf(vdist2(a, b));
}

/* Normalize a vector */
static inline vec_t vnormalize(vec_t v) {
    return vdiv(v, vmag(v));
}

/* Clamp vector norm if it exceeds the given maximum */
static inline vec_t vclampnorm(vec_t v, f32 maxnorm) {
    f32 norm = vmag(v);
    if (norm > maxnorm) {
        return vscl(v, maxnorm / norm);
    }
    return v;
}

/* Cross product of two vectors */
static inline vec_t vcross(vec_t a, vec_t b) {
    return vnew(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/* Project vector a onto unit vector b */
static inline vec_t vprojectunit(vec_t a, vec_t b_unit) {
    return vscl(b_unit, vdot(a, b_unit));
}

/* Orthogonal component of a relative to unit vector b */
static inline vec_t vorthunit(vec_t a, vec_t b_unit) {
    return vsub(a, vprojectunit(a, b_unit));
}

/* Element-wise absolute value of a vector */
static inline vec_t vabs(vec_t v) {
    return vnew(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

/* Element-wise minimum of two vectors */
static inline vec_t vmin(vec_t a, vec_t b) {
    return vnew(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}

/* Element-wise maximum of two vectors */
static inline vec_t vmax(vec_t a, vec_t b) {
    return vnew(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

/* Element-wise clamp of vector */
static inline vec_t vclamp(vec_t v, vec_t lower, vec_t upper) {
    return vmin(upper, vmax(v, lower));
}

/* Rotate a 2D vector around the z-axis */
static inline vec_t vrot2(vec_t v, f32 B) {
    return vnew(cosf(B) * v.x - sinf(B) * v.y, sinf(B) * v.x + cosf(B) * v.y, v.z);
}

/* Compute square root of a vector */
static inline vec_t vsqrt(vec_t v) {
    return vnew(sqrtf(v.x), sqrtf(v.y), sqrtf(v.z));
}

/* Square a vector */
static inline vec_t vec2(vec_t v) {
    return veltmul(v, v);
}

/* Maximum element in a vector */
static inline f32 vmaxelt(vec_t v) {
    return fmax(fmax(v.x, v.y), v.z);
}

/* Minimum element in a vector */
static inline f32 vminelt(vec_t v) {
    return fmin(fmin(v.x, v.y), v.z);
}

/* L1 norm (aka Minkowski, Taxicab, Manhattan norm) of a vector */
static inline f32 vnorm1(vec_t v) {
    return fabsf(v.x) + fabsf(v.y) + fabsf(v.z);
}

/* Compare two vectors for exact equality */
static inline bool veq(vec_t a, vec_t b) {
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

/* Compare two vectors for exact inequality */
static inline bool vneq(vec_t a, vec_t b) {
    return !veq(a, b);
}

/* Compare two vectors for near-equality with user-defined threshold */
static inline bool vequal(vec_t a, vec_t b, f32 epsilon) {
    vec_t diffs = vabs(vsub(a, b));
    return diffs.x < epsilon && diffs.y < epsilon && diffs.z < epsilon;
}

/* Element-wise less-than comparison (all elements) */
static inline bool vless(vec_t a, vec_t b) {
    return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}

/* Element-wise less-than-or-equal comparison (all elements) */
static inline bool vleq(vec_t a, vec_t b) {
    return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
}

/* Element-wise greater-than comparison (all elements) */
static inline bool vgreater(vec_t a, vec_t b) {
    return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}

/* Element-wise greater-than-or-equal comparison (all elements) */
static inline bool vgeq(vec_t a, vec_t b) {
    return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
}

/* Test if any element of a vector is NaN */
static inline bool visnan(vec_t v) {
    return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

/* Add 3 vectors */
static inline vec_t vadd3(vec_t a, vec_t b, vec_t c) {
    return vadd(vadd(a, b), c);
}

/* Add 4 vectors */
static inline vec_t vadd4(vec_t a, vec_t b, vec_t c, vec_t d) {
    /* TODO: make sure it compiles to optimal code */
    return vadd(vadd(a, b), vadd(c, d));
}

/* Subtract b and c from a */
static inline vec_t vsub2(vec_t a, vec_t b, vec_t c) {
    return vadd3(a, vneg(b), vneg(c));
}

/* Load a vector from a f64 array */
static inline vec_t vload(f64 const *d) {
    return vnew(d[0], d[1], d[2]);
}

/* Store a vector into a f64 array */
static inline void vstore(vec_t v, f64 *d) {
    d[0] = (f64)v.x;
    d[1] = (f64)v.y;
    d[2] = (f64)v.z;
}

/* Load a vector from a f32 array */
static inline vec_t vloadf(f32 const *f) {
    return vnew(f[0], f[1], f[2]);
}

/* Store a vector into a f32 array */
static inline void vstoref(vec_t v, f32 *f) {
    f[0] = v.x;
    f[1] = v.y;
    f[2] = v.z;
}

/* Index a vector like a 3-element array */
static inline f32 vindex(vec_t v, int i) {
    return ((f32 const *)&v.x)[i];
}

/* Calculate the mean of a vector */
static inline vec_t vmean(vec_t pmean, vec_t new, f32 i) {
    return vadd(vscl(pmean, i / (i + 1)), vscl(new, 1 / (i + 1)));
}

/* Calculate the variance (sigma squared) of a vector */
static inline vec_t vsigma2(vec_t psigma, vec_t mean, vec_t new, f32 i) {
    return vadd(vscl(psigma, i / (i + 1)), vscl(vec2(vsub(new, mean)), 1 / (i + 1)));
}

#endif