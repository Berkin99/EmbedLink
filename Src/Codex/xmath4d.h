#ifndef XMATH4D_H_
#define XMATH4D_H_

#include <stdint.h>
#include <stddef.h>
#include <sysdefs.h>
#include <xmath_types.h>
#include <xmath3d.h>

/* 4. Quaternions */
typedef struct {
    union {
        struct {
            f32 x;
            f32 y;
            f32 z;
        };
        f32 axis[3];
        v3f32_t v;
    };
    f32 w;
}q4f32_t;

typedef struct {
    union {
        struct {
            f64 x;
            f64 y;
            f64 z;
        };
        f64 axis[3];
        v3f64_t v;
    };
    f64 w;
}q4f64_t;

typedef q4f32_t quat_t;

/* Construct a quaternion from its x, y, z, w elements */
static inline quat_t quatnew(f32 x, f32 y, f32 z, f32 w) {
    return (quat_t){x, y, z, w};
}

/* Construct a quaternion from a vector containing (x, y, z) and a scalar w. */
static inline quat_t quatvw(vec_t v, f32 w) {
    quat_t q;
    q.x = v.x;
    q.y = v.y;
    q.z = v.z;
    q.w = w;
    return q;
}

/* Construct an identity quaternion */
static inline quat_t qeye(void) {
    return quatnew(0, 0, 0, 1);
}

/** 
 * @brief Construct a quaternion from an axis and angle of rotation. 
 *        Does not assume axis is normalized 
 */
static inline quat_t qaxisangle(vec_t axis, f32 angle) {
    f32 scale = sinf(angle / 2) / vmag(axis);
    quat_t q;
    q.x = scale * axis.x;
    q.y = scale * axis.y;
    q.z = scale * axis.z;
    q.w = cosf(angle / 2);
    return q;
}

// Forward declaration, needed in some constructors
static inline quat_t qnormalize(quat_t q);

/**
 * Construct a quaternion such that q * a = b,
 * and the rotation axis is orthogonal to the plane defined by a and b,
 * and the rotation is less than 180 degrees.
 * Assumes a and b are unit vectors.
 * Does not handle degenerate case where a = -b. Returns all-zero quaternion 
 */
static inline quat_t qvectovec(vec_t a, vec_t b) {
    vec_t const cross = vcross(a, b);
    f32 const sinangle = vmag(cross);
    f32 const cosangle = vdot(a, b);
    /* Avoid taking sqrt of negative number due to floating point error.
       TODO: find tighter exact bound */
    f32 const EPS_ANGLE = 1e-6;
    if (sinangle < EPS_ANGLE) {
        if (cosangle > 0.0f) return qeye();
        else return quatnew(0.0f, 0.0f, 0.0f, 0.0f); /* Degenerate case */
    }
    f32 const halfcos = 0.5f * cosangle;
    /* Since angle is < 180 degrees, the positive sqrt is always correct */
    f32 const sinhalfangle = sqrtf(fmax(0.5f - halfcos, 0.0f));
    f32 const coshalfangle = sqrtf(fmax(0.5f + halfcos, 0.0f));
    vec_t const qimag = vscl(cross, sinhalfangle / sinangle);
    f32 const qreal = coshalfangle;
    return quatvw(qimag, qreal);
}

/* Construct from (roll, pitch, yaw) Euler angles using Tait-Bryan convention
   (yaw, then pitch about new pitch axis, then roll about new roll axis) */
static inline quat_t rpy2quat(vec_t rpy) {
    // from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    f32 r = rpy.x;
    f32 p = rpy.y;
    f32 y = rpy.z;
    f32 cr = cosf(r / 2.0f);
    f32 sr = sinf(r / 2.0f);
    f32 cp = cosf(p / 2.0f);
    f32 sp = sinf(p / 2.0f);
    f32 cy = cosf(y / 2.0f);
    f32 sy = sinf(y / 2.0f);

    f32 qx = sr * cp * cy - cr * sp * sy;
    f32 qy = cr * sp * cy + sr * cp * sy;
    f32 qz = cr * cp * sy - sr * sp * cy;
    f32 qw = cr * cp * cy + sr * sp * sy;

    return quatnew(qx, qy, qz, qw);
}



/* APPROXIMATE construction of a quaternion from small (roll, pitch, yaw) Euler angles
   without computing any trig functions. Only produces useful results for small angles.
   Example application is integrating a gyroscope when the angular velocity
   of the object is small compared to the sampling frequency. */
static inline quat_t rpy2quat_small(vec_t rpy) {
    /* TODO: cite source, but can be derived from rpy2quat under first-order approximation:
       sin(epsilon) = epsilon, cos(epsilon) = 1, epsilon^2 = 0 */
    f32 q2 = vmag2(rpy) / 4.0f;
    if (q2 < 1) {
        return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
    } else {
        f32 w = 1.0f / sqrtf(1.0f + q2);
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
    f32 s = 1.0f / sqrtf(1.0f - q.w * q.w);
    return vscl(vnew(q.x, q.y, q.z), s);
}

/* Compute the angle of a quaternion's axis-angle decomposition.
   Result lies in the domain (-pi, pi]. */
static inline f32 quat2angle(quat_t q) {
    f32 angle = 2 * acosf(q.w);
    if (angle > M_PI_F32) {
        angle -= 2.0f * M_PI_F32;
    }
    return angle;
}

/* Vector containing the imaginary part of the quaternion, i.e. (x, y, z) */
static inline vec_t quatimagpart(quat_t q) {
    return vnew(q.x, q.y, q.z);
}

/* Rotate a vector by a quaternion. */
static inline vec_t qvrot(quat_t q, vec_t v) {
    vec_t qv = vnew(q.x, q.y, q.z);
    return vadd3(
        vscl(qv, 2.0f * vdot(qv, v)),
        vscl(v, q.w * q.w - vmag2(qv)),
        vscl(vcross(qv, v), 2.0f * q.w)
    );
}

/* Multiply (compose) two quaternions such that
   qvrot(qqmul(q, p), v) == qvrot(q, qvrot(p, v)). */
static inline quat_t qqmul(quat_t q, quat_t p) {
    f32 x =  q.w * p.x + q.z * p.y - q.y * p.z + q.x * p.w;
    f32 y = -q.z * p.x + q.w * p.y + q.x * p.z + q.y * p.w;
    f32 z =  q.y * p.x - q.x * p.y + q.w * p.z + q.z * p.w;
    f32 w = -q.x * p.x - q.y * p.y - q.z * p.z + q.w * p.w;
    return quatnew(x, y, z, w);
}

/* Invert a quaternion. */
static inline quat_t qinv(quat_t q) {
    return quatnew(-q.x, -q.y, -q.z, q.w);
}

/* Negate a quaternion.
   This represents the same rotation, but is still sometimes useful. */
static inline quat_t qneg(quat_t q) {
    return quatnew(-q.x, -q.y, -q.z, -q.w);
}

/* Return a quaternion representing the same rotation
   but with a positive real term (q.w).
   Useful to collapse the f64-covering of SO(3) by the quaternions. */
static inline quat_t qposreal(quat_t q) {
    if (q.w < 0) return qneg(q);
    return q;
}

/* Quaternion dot product. Is cosine of angle between them. */
static inline f32 qdot(quat_t a, quat_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline f32 qanglebetween(quat_t a, quat_t b) {
    f32 const dot = qdot(qposreal(a), qposreal(b));
    /* Prevent acos domain issues */
    if (dot > 1.0f - 1e-9f) return 0.0f;
    if (dot < -1.0f + 1e-9f) return M_PI_F32;
    return acosf(dot);
}

static inline bool qeq(quat_t a, quat_t b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

/* Normalize a quaternion.
   Typically used to mitigate precision errors. */
static inline quat_t qnormalize(quat_t q) {
    f32 s = 1.0f / sqrtf(qdot(q, q));
    return quatnew(s * q.x, s * q.y, s * q.z, s * q.w);
}

/* Update an attitude estimate quaternion with a reading from a gyroscope
   over the timespan dt. Gyroscope is assumed (roll, pitch, yaw)
   angular velocities in radians per second. */
static inline quat_t quat_gyro_update(quat_t quat, vec_t gyro, f32 const dt) {
    /* From "Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005 */
    quat_t q1;
    f32 const r = (dt / 2) * gyro.x;
    f32 const p = (dt / 2) * gyro.y;
    f32 const y = (dt / 2) * gyro.z;

    q1.x =    quat.x + y * quat.y - p * quat.z + r * quat.w;
    q1.y = -y * quat.x +   quat.y + r * quat.z + p * quat.w;
    q1.z =  p * quat.x - r * quat.y +   quat.z + y * quat.w;
    q1.w = -r * quat.x - p * quat.y - y * quat.z +   quat.w;
    return q1;
}

/* Normalized linear interpolation. s should be between 0 and 1. */
static inline quat_t qnlerp(quat_t a, quat_t b, f32 t) {
    f32 s = 1.0f - t;
    return qnormalize(quatnew(
        s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z, s * a.w + t * b.w));
}

/* Spherical linear interpolation. s should be between 0 and 1. */
static inline quat_t qslerp(quat_t a, quat_t b, f32 t) {
    /* From "Animating Rotation with Quaternion Curves", Ken Shoemake, 1985 */
    f32 dp = qdot(a, b);
    if (dp < 0) {
        dp = -dp;
        b = qneg(b);
    }

    if (dp > 0.99f) {
        /* Fall back to linear interpolation to avoid div-by-zero */
        return qnlerp(a, b, t);
    } else {
        f32 theta = acosf(dp);
        f32 s = sinf(theta * (1 - t)) / sinf(theta);
        t = sinf(theta * t) / sinf(theta);
        return quatnew(
            s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z, s * a.w + t * b.w);
    }
}

/* Load a quaternion from a raw f64 array. */
static inline quat_t qload(f64 const *d) {
    return quatnew(d[0], d[1], d[2], d[3]);
}

/* Store a quaternion into a raw f64 array. */
static inline void qstore(quat_t q, f64 *d) {
    d[0] = (f64)q.x; d[1] = (f64)q.y; d[2] = (f64)q.z; d[3] = (f64)q.w;
}

/* Load a quaternion from a raw f32 array. */
static inline quat_t qloadf(f32 const *f) {
    return quatnew(f[0], f[1], f[2], f[3]);
}

/* Store a quaternion into a raw f32 array. */
static inline void qstoref(quat_t q, f32 *f) {
    f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}

#endif
 