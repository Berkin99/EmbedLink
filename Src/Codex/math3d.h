#ifndef MATH3D_H_
#define MATH3D_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define M_PI_F     (3.14159265358979323846f)
#define M_1_PI_F   (0.31830988618379067154f)
#define M_PI_2_F   (1.57079632679f)
#define RAD2DEG    (180.0 / M_PI_F)
#define DEG2RAD    (M_PI_F / 180.0)

/* 3-axis 16-bit vector type */
typedef union {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} vec16_t;

/* 3-axis 32-bit vector type */
typedef union {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
    };
    int32_t axis[3];
} vec32_t;

/* 3-axis 64-bit vector type */
typedef union {
    struct {
        int64_t x;
        int64_t y;
        int64_t z;
    };
    int64_t axis[3];
} vec64_t;

/* 3-axis floating-point vector type */
typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} vec_t;

/* Return square of a float */
static inline float fsqr(float x) { return x * x; }

/* Convert degrees to radians */
static inline float radians(float degrees) { return (M_PI_F / 180.0f) * degrees; }

/* Convert radians to degrees */
static inline float degrees(float radians) { return (180.0f / M_PI_F) * radians; }

/* Normalize radians to range [-pi, pi] */
static inline float normalize_radians(float radians)
{
    float signed_pi = copysignf(M_PI_F, radians);
    radians = fmodf(radians + signed_pi, 2 * M_PI_F) - signed_pi;
    return radians;
}

/* Modulo operation using floored definition */
static inline float fmodf_floored(float x, float n)
{
    return x - floorf(x / n) * n;
}

/* Compute the shortest signed angle between two given angles in radians */
static inline float shortest_signed_angle_radians(float start, float goal)
{
    float diff = goal - start;
    float signed_diff = fmodf_floored(diff + M_PI_F, 2 * M_PI_F) - M_PI_F;
    return signed_diff;
}

/* Clamp a float value between a min and max value */
static inline float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/* Compare two floats for approximate equality with ulps threshold */
static inline bool fcloseulps(float a, float b, int ulps) {
    if ((a < 0.0f) != (b < 0.0f)) {
        if (a == b) {
            return true;
        }
        return false;
    }
    int ia = *((int *)&a);
    int ib = *((int *)&b);
    return fabsf(ia - ib) <= ulps;
}

/* Construct a vector from 3 floats */
static inline vec_t mkvec(float x, float y, float z) {
    vec_t v;
    v.x = x; v.y = y; v.z = z;
    return v;
}

/* Construct a vector with the same value repeated */
static inline vec_t vrepeat(float x) {
    return mkvec(x, x, x);
}

/* Construct a zero vector */
static inline vec_t vzero(void) {
    return vrepeat(0.0f);
}

/* Construct the i'th basis vector */
static inline vec_t vbasis(int i) {
    float a[3] = {0.0f, 0.0f, 0.0f};
    a[i] = 1.0f;
    return mkvec(a[0], a[1], a[2]);
}

/* Multiply a vector by a scalar */
static inline vec_t vscl(vec_t v, float s) {
    return mkvec(s * v.x , s * v.y, s * v.z);
}

/* Negate a vector */
static inline vec_t vneg(vec_t v) {
    return mkvec(-v.x, -v.y, -v.z);
}

/* Divide a vector by a scalar */
static inline vec_t vdiv(vec_t v, float s) {
    return vscl(v, 1.0f/s);
}

/* Add two vectors */
static inline vec_t vadd(vec_t a, vec_t b) {
    return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}

/* Subtract two vectors */
static inline vec_t vsub(vec_t a, vec_t b) {
    return vadd(a, vneg(b));
}

/* Dot product of two vectors */
static inline float vdot(vec_t a, vec_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* Element-wise vector multiply */
static inline vec_t veltmul(vec_t a, vec_t b) {
    return mkvec(a.x * b.x, a.y * b.y, a.z * b.z);
}

/* Element-wise vector divide */
static inline vec_t veltdiv(vec_t a, vec_t b) {
    return mkvec(a.x / b.x, a.y / b.y, a.z / b.z);
}

/* Element-wise vector reciprocal */
static inline vec_t veltrecip(vec_t a) {
    return mkvec(1.0f / a.x, 1.0f / a.y, 1.0f / a.z);
}

/* Vector magnitude squared */
static inline float vmag2(vec_t v) {
    return vdot(v, v);
}

/* Vector magnitude */
static inline float vmag(vec_t v) {
    return sqrtf(vmag2(v));
}

/* Euclidean distance squared between two vectors */
static inline float vdist2(vec_t a, vec_t b) {
    return vmag2(vsub(a, b));
}

/* Euclidean distance between two vectors */
static inline float vdist(vec_t a, vec_t b) {
    return sqrtf(vdist2(a, b));
}

/* Normalize a vector */
static inline vec_t vnormalize(vec_t v) {
    return vdiv(v, vmag(v));
}

/* Clamp vector norm if it exceeds the given maximum */
static inline vec_t vclampnorm(vec_t v, float maxnorm) {
    float norm = vmag(v);
    if (norm > maxnorm) {
        return vscl(v, maxnorm / norm);
    }
    return v;
}

/* Cross product of two vectors */
static inline vec_t vcross(vec_t a, vec_t b) {
    return mkvec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
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
    return mkvec(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

/* Element-wise minimum of two vectors */
static inline vec_t vmin(vec_t a, vec_t b) {
    return mkvec(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}

/* Element-wise maximum of two vectors */
static inline vec_t vmax(vec_t a, vec_t b) {
    return mkvec(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

/* Element-wise clamp of vector */
static inline vec_t vclamp(vec_t v, vec_t lower, vec_t upper) {
    return vmin(upper, vmax(v, lower));
}

/* Rotate a 2D vector around the z-axis */
static inline vec_t vrot2(vec_t v, float B) {
    return mkvec(cosf(B) * v.x - sinf(B) * v.y, sinf(B) * v.x + cosf(B) * v.y, v.z);
}

/* Compute square root of a vector */
static inline vec_t vsqrt(vec_t v) {
    return mkvec(sqrtf(v.x), sqrtf(v.y), sqrtf(v.z));
}

/* Square a vector */
static inline vec_t vec2(vec_t v) {
    return veltmul(v, v);
}

/* Maximum element in a vector */
static inline float vmaxelt(vec_t v) {
    return fmax(fmax(v.x, v.y), v.z);
}

/* Minimum element in a vector */
static inline float vminelt(vec_t v) {
    return fmin(fmin(v.x, v.y), v.z);
}

/* L1 norm (aka Minkowski, Taxicab, Manhattan norm) of a vector */
static inline float vnorm1(vec_t v) {
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
static inline bool veqepsilon(vec_t a, vec_t b, float epsilon) {
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

/* Load a vector from a double array */
static inline vec_t vload(double const *d) {
    return mkvec(d[0], d[1], d[2]);
}

/* Store a vector into a double array */
static inline void vstore(vec_t v, double *d) {
    d[0] = (double)v.x;
    d[1] = (double)v.y;
    d[2] = (double)v.z;
}

/* Load a vector from a float array */
static inline vec_t vloadf(float const *f) {
    return mkvec(f[0], f[1], f[2]);
}

/* Store a vector into a float array */
static inline void vstoref(vec_t v, float *f) {
    f[0] = v.x;
    f[1] = v.y;
    f[2] = v.z;
}

/* Index a vector like a 3-element array */
static inline float vindex(vec_t v, int i) {
    return ((float const *)&v.x)[i];
}

/* Calculate the mean of a vector */
static inline vec_t vmean(vec_t pmean, vec_t new, float i) {
    return vadd(vscl(pmean, i / (i + 1)), vscl(new, 1 / (i + 1)));
}

/* Calculate the variance (sigma squared) of a vector */
static inline vec_t vsigma2(vec_t psigma, vec_t mean, vec_t new, float i) {
    return vadd(vscl(psigma, i / (i + 1)), vscl(vec2(vsub(new, mean)), 1 / (i + 1)));
}


/* Quaternion structure */
typedef struct {
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float axis[3]; /* Array representation of the vector part */
    };
    float w;
}quat_t;

/* Construct a quaternion from its x, y, z, w elements */
static inline quat_t mkquat(float x, float y, float z, float w) {
    quat_t q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
}

/* Construct a quaternion from a vector containing (x, y, z) and a scalar w. */
static inline quat_t quatvw(vec_t v, float w) {
    quat_t q;
    q.x = v.x;
    q.y = v.y;
    q.z = v.z;
    q.w = w;
    return q;
}

/* Construct an identity quaternion */
static inline quat_t qeye(void) {
    return mkquat(0, 0, 0, 1);
}

/* Construct a quaternion from an axis and angle of rotation.
   Does not assume axis is normalized */
static inline quat_t qaxisangle(vec_t axis, float angle) {
    float scale = sinf(angle / 2) / vmag(axis);
    quat_t q;
    q.x = scale * axis.x;
    q.y = scale * axis.y;
    q.z = scale * axis.z;
    q.w = cosf(angle / 2);
    return q;
}

// Forward declaration, needed in some constructors
static inline quat_t qnormalize(quat_t q);

/* Construct a quaternion such that q * a = b,
   and the rotation axis is orthogonal to the plane defined by a and b,
   and the rotation is less than 180 degrees.
   Assumes a and b are unit vectors.
   Does not handle degenerate case where a = -b. Returns all-zero quaternion */
static inline quat_t qvectovec(vec_t a, vec_t b) {
    vec_t const cross = vcross(a, b);
    float const sinangle = vmag(cross);
    float const cosangle = vdot(a, b);
    /* Avoid taking sqrt of negative number due to floating point error.
       TODO: find tighter exact bound */
    float const EPS_ANGLE = 1e-6;
    if (sinangle < EPS_ANGLE) {
        if (cosangle > 0.0f) return qeye();
        else return mkquat(0.0f, 0.0f, 0.0f, 0.0f); /* Degenerate case */
    }
    float const halfcos = 0.5f * cosangle;
    /* Since angle is < 180 degrees, the positive sqrt is always correct */
    float const sinhalfangle = sqrtf(fmax(0.5f - halfcos, 0.0f));
    float const coshalfangle = sqrtf(fmax(0.5f + halfcos, 0.0f));
    vec_t const qimag = vscl(cross, sinhalfangle / sinangle);
    float const qreal = coshalfangle;
    return quatvw(qimag, qreal);
}

/* Construct from (roll, pitch, yaw) Euler angles using Tait-Bryan convention
   (yaw, then pitch about new pitch axis, then roll about new roll axis) */
static inline quat_t rpy2quat(vec_t rpy) {
    // from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float r = rpy.x;
    float p = rpy.y;
    float y = rpy.z;
    float cr = cosf(r / 2.0f);
    float sr = sinf(r / 2.0f);
    float cp = cosf(p / 2.0f);
    float sp = sinf(p / 2.0f);
    float cy = cosf(y / 2.0f);
    float sy = sinf(y / 2.0f);

    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;
    float qw = cr * cp * cy + sr * sp * sy;

    return mkquat(qx, qy, qz, qw);
}



/* APPROXIMATE construction of a quaternion from small (roll, pitch, yaw) Euler angles
   without computing any trig functions. Only produces useful results for small angles.
   Example application is integrating a gyroscope when the angular velocity
   of the object is small compared to the sampling frequency. */
static inline quat_t rpy2quat_small(vec_t rpy) {
    /* TODO: cite source, but can be derived from rpy2quat under first-order approximation:
       sin(epsilon) = epsilon, cos(epsilon) = 1, epsilon^2 = 0 */
    float q2 = vmag2(rpy) / 4.0f;
    if (q2 < 1) {
        return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
    } else {
        float w = 1.0f / sqrtf(1.0f + q2);
        return quatvw(vscl(rpy, w / 2), w);
    }
}

/* Conversions to other parameterizations of 3D rotations */

/* Convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention
   (yaw, then pitch about new pitch axis, then roll about new roll axis) */
static inline vec_t quat2rpy(quat_t q) {
    /* from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles */
    vec_t v;
    v.x = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1 - 2 * (fsqr(q.x) + fsqr(q.y))); /* roll */
    v.y = asinf(2.0f * (q.w * q.y - q.x * q.z)); /* pitch */
    v.z = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1 - 2 * (fsqr(q.y) + fsqr(q.z))); /* yaw */
    return v;
}

/* Compute the axis of a quaternion's axis-angle decomposition. */
static inline vec_t quat2axis(quat_t q) {
    /* TODO: this is not numerically stable for tiny rotations */
    float s = 1.0f / sqrtf(1.0f - q.w * q.w);
    return vscl(mkvec(q.x, q.y, q.z), s);
}

/* Compute the angle of a quaternion's axis-angle decomposition.
   Result lies in the domain (-pi, pi]. */
static inline float quat2angle(quat_t q) {
    float angle = 2 * acosf(q.w);
    if (angle > M_PI_F) {
        angle -= 2.0f * M_PI_F;
    }
    return angle;
}

/* Vector containing the imaginary part of the quaternion, i.e. (x, y, z) */
static inline vec_t quatimagpart(quat_t q) {
    return mkvec(q.x, q.y, q.z);
}

/* Rotate a vector by a quaternion. */
static inline vec_t qvrot(quat_t q, vec_t v) {
    vec_t qv = mkvec(q.x, q.y, q.z);
    return vadd3(
        vscl(qv, 2.0f * vdot(qv, v)),
        vscl(v, q.w * q.w - vmag2(qv)),
        vscl(vcross(qv, v), 2.0f * q.w)
    );
}

/* Multiply (compose) two quaternions such that
   qvrot(qqmul(q, p), v) == qvrot(q, qvrot(p, v)). */
static inline quat_t qqmul(quat_t q, quat_t p) {
    float x =  q.w * p.x + q.z * p.y - q.y * p.z + q.x * p.w;
    float y = -q.z * p.x + q.w * p.y + q.x * p.z + q.y * p.w;
    float z =  q.y * p.x - q.x * p.y + q.w * p.z + q.z * p.w;
    float w = -q.x * p.x - q.y * p.y - q.z * p.z + q.w * p.w;
    return mkquat(x, y, z, w);
}

/* Invert a quaternion. */
static inline quat_t qinv(quat_t q) {
    return mkquat(-q.x, -q.y, -q.z, q.w);
}

/* Negate a quaternion.
   This represents the same rotation, but is still sometimes useful. */
static inline quat_t qneg(quat_t q) {
    return mkquat(-q.x, -q.y, -q.z, -q.w);
}

/* Return a quaternion representing the same rotation
   but with a positive real term (q.w).
   Useful to collapse the double-covering of SO(3) by the quaternions. */
static inline quat_t qposreal(quat_t q) {
    if (q.w < 0) return qneg(q);
    return q;
}

/* Quaternion dot product. Is cosine of angle between them. */
static inline float qdot(quat_t a, quat_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline float qanglebetween(quat_t a, quat_t b) {
    float const dot = qdot(qposreal(a), qposreal(b));
    /* Prevent acos domain issues */
    if (dot > 1.0f - 1e-9f) return 0.0f;
    if (dot < -1.0f + 1e-9f) return M_PI_F;
    return acosf(dot);
}

static inline bool qeq(quat_t a, quat_t b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

/* Normalize a quaternion.
   Typically used to mitigate precision errors. */
static inline quat_t qnormalize(quat_t q) {
    float s = 1.0f / sqrtf(qdot(q, q));
    return mkquat(s * q.x, s * q.y, s * q.z, s * q.w);
}

/* Update an attitude estimate quaternion with a reading from a gyroscope
   over the timespan dt. Gyroscope is assumed (roll, pitch, yaw)
   angular velocities in radians per second. */
static inline quat_t quat_gyro_update(quat_t quat, vec_t gyro, float const dt) {
    /* From "Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005 */
    quat_t q1;
    float const r = (dt / 2) * gyro.x;
    float const p = (dt / 2) * gyro.y;
    float const y = (dt / 2) * gyro.z;

    q1.x =    quat.x + y * quat.y - p * quat.z + r * quat.w;
    q1.y = -y * quat.x +   quat.y + r * quat.z + p * quat.w;
    q1.z =  p * quat.x - r * quat.y +   quat.z + y * quat.w;
    q1.w = -r * quat.x - p * quat.y - y * quat.z +   quat.w;
    return q1;
}

/* Normalized linear interpolation. s should be between 0 and 1. */
static inline quat_t qnlerp(quat_t a, quat_t b, float t) {
    float s = 1.0f - t;
    return qnormalize(mkquat(
        s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z, s * a.w + t * b.w));
}

/* Spherical linear interpolation. s should be between 0 and 1. */
static inline quat_t qslerp(quat_t a, quat_t b, float t) {
    /* From "Animating Rotation with Quaternion Curves", Ken Shoemake, 1985 */
    float dp = qdot(a, b);
    if (dp < 0) {
        dp = -dp;
        b = qneg(b);
    }

    if (dp > 0.99f) {
        /* Fall back to linear interpolation to avoid div-by-zero */
        return qnlerp(a, b, t);
    } else {
        float theta = acosf(dp);
        float s = sinf(theta * (1 - t)) / sinf(theta);
        t = sinf(theta * t) / sinf(theta);
        return mkquat(
            s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z, s * a.w + t * b.w);
    }
}

/* Load a quaternion from a raw double array. */
static inline quat_t qload(double const *d) {
    return mkquat(d[0], d[1], d[2], d[3]);
}

/* Store a quaternion into a raw double array. */
static inline void qstore(quat_t q, double *d) {
    d[0] = (double)q.x; d[1] = (double)q.y; d[2] = (double)q.z; d[3] = (double)q.w;
}

/* Load a quaternion from a raw float array. */
static inline quat_t qloadf(float const *f) {
    return mkquat(f[0], f[1], f[2], f[3]);
}

/* Store a quaternion into a raw float array. */
static inline void qstoref(quat_t q, float *f) {
    f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}

#ifdef __cplusplus
}
#endif

#endif /* MATH3D_H_ */
