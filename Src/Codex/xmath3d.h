#ifndef XMATH3D_H_
#define XMATH3D_H_

#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <xmath_types.h>

typedef v3f32_t vec_t;

static inline vec_t vnew(f32 x, f32 y, f32 z) {
    return (vec_t){ .x = x, .y = y, .z = z };
}

static inline vec_t vneg(vec_t v) {
    return (vec_t){ .x = -v.x, .y = -v.y, .z = -v.z };
}

static inline vec_t vadd(vec_t a, vec_t b) {
    return (vec_t){ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
}

static inline vec_t vsub(vec_t a, vec_t b) {
    return vadd(a, vneg(b));
}

static inline vec_t vscl(vec_t v, f32 s) {
    return (vec_t){ .x = s * v.x, .y = s * v.y, .z = s * v.z };
}

static inline vec_t vdiv(vec_t v, f32 s) {
    return vscl(v, 1.0f / s);
}

static inline f32 vdot(vec_t a, vec_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline vec_t vrepeat(f32 x) {
    return (vec_t){ .x = x, .y = x, .z = x };
}

static inline vec_t vzero(void) {
    return vrepeat(0.0f);
}

static inline vec_t vbasis(int i) {
    f32 a[3] = {0.0f, 0.0f, 0.0f};
    a[i] = 1.0f;
    return (vec_t){ .x = a[0], .y = a[1], .z = a[2] };
}

static inline vec_t veltmul(vec_t a, vec_t b) {
    return (vec_t){ .x = a.x * b.x, .y = a.y * b.y, .z = a.z * b.z };
}

static inline vec_t veltdiv(vec_t a, vec_t b) {
    return (vec_t){ .x = a.x / b.x, .y = a.y / b.y, .z = a.z / b.z };
}

static inline vec_t veltrecip(vec_t a) {
    return (vec_t){ .x = 1.0f / a.x, .y = 1.0f / a.y, .z = 1.0f / a.z };
}

static inline f32 vmag2(vec_t v) {
    return vdot(v, v);
}

static inline f32 vmag(vec_t v) {
    return sqrtf(vmag2(v));
}

static inline f32 vdist2(vec_t a, vec_t b) {
    return vmag2(vsub(a, b));
}

static inline f32 vdist(vec_t a, vec_t b) {
    return sqrtf(vdist2(a, b));
}

static inline vec_t vnormalize(vec_t v) {
    return vdiv(v, vmag(v));
}

static inline vec_t vclampnorm(vec_t v, f32 maxnorm) {
    f32 norm = vmag(v);
    if (norm > maxnorm) {
        return vscl(v, maxnorm / norm);
    }
    return v;
}

static inline vec_t vcross(vec_t a, vec_t b) {
    return (vec_t){
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };
}

static inline vec_t vprojectunit(vec_t a, vec_t b_unit) {
    return vscl(b_unit, vdot(a, b_unit));
}

static inline vec_t vorthunit(vec_t a, vec_t b_unit) {
    return vsub(a, vprojectunit(a, b_unit));
}

static inline vec_t vabs(vec_t v) {
    return (vec_t){ .x = fabsf(v.x), .y = fabsf(v.y), .z = fabsf(v.z) };
}

static inline vec_t vmin(vec_t a, vec_t b) {
    return (vec_t){ .x = fminf(a.x, b.x), .y = fminf(a.y, b.y), .z = fminf(a.z, b.z) };
}

static inline vec_t vmax(vec_t a, vec_t b) {
    return (vec_t){ .x = fmaxf(a.x, b.x), .y = fmaxf(a.y, b.y), .z = fmaxf(a.z, b.z) };
}

static inline vec_t vclamp(vec_t v, vec_t lower, vec_t upper) {
    return vmin(upper, vmax(v, lower));
}

static inline vec_t vrot2(vec_t v, f32 B) {
    return (vec_t){
        .x = cosf(B) * v.x - sinf(B) * v.y,
        .y = sinf(B) * v.x + cosf(B) * v.y,
        .z = v.z
    };
}

static inline vec_t vsqrt(vec_t v) {
    return (vec_t){ .x = sqrtf(v.x), .y = sqrtf(v.y), .z = sqrtf(v.z) };
}

static inline vec_t vec2(vec_t v) {
    return veltmul(v, v);
}

static inline f32 vmaxelt(vec_t v) {
    return fmaxf(fmaxf(v.x, v.y), v.z);
}

static inline f32 vminelt(vec_t v) {
    return fminf(fminf(v.x, v.y), v.z);
}

static inline f32 vnorm1(vec_t v) {
    return fabsf(v.x) + fabsf(v.y) + fabsf(v.z);
}

static inline bool veq(vec_t a, vec_t b) {
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

static inline bool vneq(vec_t a, vec_t b) {
    return !veq(a, b);
}

static inline bool vequal(vec_t a, vec_t b, f32 epsilon) {
    vec_t diffs = vabs(vsub(a, b));
    return diffs.x < epsilon && diffs.y < epsilon && diffs.z < epsilon;
}

static inline bool vless(vec_t a, vec_t b) {
    return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}

static inline bool vleq(vec_t a, vec_t b) {
    return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
}

static inline bool vgreater(vec_t a, vec_t b) {
    return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}

static inline bool vgeq(vec_t a, vec_t b) {
    return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
}

static inline bool visnan(vec_t v) {
    return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

static inline vec_t vadd3(vec_t a, vec_t b, vec_t c) {
    return vadd(vadd(a, b), c);
}

static inline vec_t vadd4(vec_t a, vec_t b, vec_t c, vec_t d) {
    return vadd(vadd(a, b), vadd(c, d));
}

static inline vec_t vsub2(vec_t a, vec_t b, vec_t c) {
    return vadd3(a, vneg(b), vneg(c));
}

static inline vec_t vload(f32 const *d) {
    return (vec_t){ .x = d[0], .y = d[1], .z = d[2] };
}

static inline void vstore(vec_t v, f32 *d) {
    d[0] = v.x;
    d[1] = v.y;
    d[2] = v.z;
}

static inline vec_t vloadf(f32 const *f) {
    return (vec_t){ .x = f[0], .y = f[1], .z = f[2] };
}

static inline void vstoref(vec_t v, f32 *f) {
    f[0] = v.x;
    f[1] = v.y;
    f[2] = v.z;
}

static inline f32 vindex(vec_t v, int i) {
    return ((f32 const *)&v.x)[i];
}

static inline vec_t vmean(vec_t pmean, vec_t new, f32 i) {
    return vadd(vscl(pmean, i / (i + 1)), vscl(new, 1 / (i + 1)));
}

static inline vec_t vsigma2(vec_t psigma, vec_t mean, vec_t new, f32 i) {
    return vadd(vscl(psigma, i / (i + 1)), vscl(vec2(vsub(new, mean)), 1 / (i + 1)));
}

#endif
