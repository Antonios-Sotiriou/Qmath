/* 
MIT License

Copyright (c) 2026 Antonios Sotiriou

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef QMATH_H
#define QMATH_H 1

#ifdef VECTORIZED_CODE
#include <xmmintrin.h>

typedef union vec4 {
    __m128 m;
    float c[4];
} vec4;

#else

typedef union vec4 {
    float c[4];
} vec4;

#endif // !VECTORIZED_CODE

/* Quaternion's internal format is W X Y Z. */
typedef vec4 quat;

typedef union {
    vec4 m[4];
} mat4x4;

quat unitQuat(void);
quat setQuat(const float angle, const float x, const float y, const float z);
float magnitudeQuat(const quat q);
void normalizeQuat(quat *q);
quat conjugateQuat(const quat q);
quat rotationQuat(const float angle, const float x, const float y, const float z);
vec4 vec4RotateQuat(const quat q, const vec4 v);
void setvec4RotateQuat(const quat q, vec4 *v);
quat addQuats(const quat q1, const quat q2);
quat eulertoQuat(const float roll, const float yaw, const float pitch);
quat multiplyQuats(const quat q1, const quat q2);
mat4x4 MatfromQuat(const quat q, const float x, const float y, const float z);
quat quatFromMat(mat4x4 m);
quat slerp(const quat q1, const quat q2, const float t);
quat lerp(const quat q1, const quat q2, const float t);

#ifdef QMATH_IMPLEMENTATION

#include <math.h>

#ifdef VECTORIZED_CODE // #######################################################################################
/* Usefull Globals to increase some calculations performance. */
// Useful for eulerToQuat;
const static quat etqor = { 0.f, -0.f, 0.f, -0.f };
// Useful for multiplyQuats;
const static quat mqor1 = { -0.f, 0.f, -0.f, 0.f };
const static quat mqor2 = { -0.f, 0.f, 0.f, -0.f };
const static quat mqor3 = { -0.f, -0.f, 0.f, 0.f };

/* Useful for matfromQuat */
const static vec4 twos = { 2.f, 2.f, 2.f, 0.f };

const static vec4 ones1 = { 1.f, 0.f, 0.f, 0.f };
const static vec4 mfqor1 = { 0.f, -0.f, 0.f, 0.f };

const static vec4 ones2 = { 0.f, 1.f, 0.f, 0.f };
const static vec4 mfqor2 = { 0.f, 0.f, -0.f, 0.f };

const static vec4 ones3 = { 0.f, 0.f, 1.f, 0.f };
const static vec4 mfqor3 = { -0.f, 0.f, 0.f, 0.f };

// Useful for slerp;
const static quat slhalf = { 0.5f, 0.5f, 0.5f, 0.5f };

/* Creates an initialized unit quaternion. */
quat unitQuat(void) {
    quat r = {
        .m = _mm_set_ss(1.f)
    };
    return r;
}
/* Sets a quat's parameters, creating a new quat. */
quat setQuat(const float angle, const float x, const float y, const float z) {
    quat r = {
        .m = _mm_setr_ps(angle, x, y, z)
    };
    return r;
}
/* Computes the magnitude aka length of a given quat. */
float magnitudeQuat(const quat q) {
    quat r = { 0 };
    r.m = _mm_mul_ps(q.m, q.m);
    return _mm_cvtss_f32(
               _mm_sqrt_ps(
                   _mm_add_ps(
                       _mm_add_ps(r.m, _mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 0, 0, 1))),
                       _mm_add_ps(_mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 0, 0, 2)), _mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 0, 0, 3)))
                   )
               )
    );
}
/* Normalizes given quat if its not already. */
void normalizeQuat(quat *q) {
    quat r = {
        .m = _mm_mul_ps(q->m, q->m)
    };
    float check = _mm_cvtss_f32(r.m) +
        _mm_cvtss_f32(_mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 3, 2, 1))) +
        _mm_cvtss_f32(_mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(1, 0, 3, 2))) +
        _mm_cvtss_f32(_mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(2, 1, 0, 3)));
    if (check > 1.000001f) {
        q->m = _mm_div_ps(q->m, _mm_sqrt_ps(_mm_set_ps1(check)));
    }
}
/* Conjugate quat's vector part, aka( switching the sign ). */
quat conjugateQuat(const quat q) {
    quat r = {
        .m = _mm_setr_ps(0.f, -0.f, -0.f, -0.f)
    };

    r.m = _mm_xor_ps(q.m, r.m);
    return r;
}
/* Creates a rotation quaternion with angle W and rotation axis X Y Z: */
quat rotationQuat(const float angle, const float x, const float y, const float z) {
    const float radius = (angle * (3.14159f / 180.0f)) * 0.5f;
    const float sn = sinf(radius);
    const float cs = cosf(radius);

    return (quat) {
        .m = _mm_mul_ps(_mm_setr_ps(cs, x, y, z), _mm_setr_ps(1.f, sn, sn, sn))
    };
}
/* Rotates vector v by the given quaternion.Returns a new vector. */
vec4 vec4RotateQuat(const quat q, const vec4 v) {
    quat r = {
        .m = _mm_shuffle_ps(v.m, v.m, _MM_SHUFFLE(2, 1, 0, 3))
    };
    r = multiplyQuats(multiplyQuats(conjugateQuat(q), r), q);

    return (vec4) {
        .m = _mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 3, 2, 1))
    };
}
/* Rotates vector v by the given quaternion. */
void setvec4RotateQuat(const quat q, vec4 *v) {
    quat r = {
        .m = _mm_shuffle_ps(v->m, v->m, _MM_SHUFFLE(2, 1, 0, 3))
    };
    r = multiplyQuats(multiplyQuats(conjugateQuat(q), r), q);
    v->m = _mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 3, 2, 1));
}
/* Adds quats q1 and q2 together returning a new quat. */
quat addQuats(const quat q1, const quat q2) {
    return (quat) {
        .m = _mm_add_ps(q1.m, q2.m)
    };
}
/* Creates a quaternion from the given euler angles. */
quat eulertoQuat(const float roll, const float yaw, const float pitch) {
    const float half_roll = roll * 0.5f;
    const float half_yaw = yaw * 0.5f;
    const float half_pitch = pitch * 0.5f;

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);

    quat crsrcrcr = {
        .m = _mm_setr_ps(cr, sr, cr, cr)
    };
    quat cpcpspcp = {
        .m = _mm_setr_ps(cp, cp, sp, cp)
    };
    quat cycycysy = {
        .m = _mm_setr_ps(cy, cy, cy, sy)
    };

    quat left = {
        .m = _mm_mul_ps(_mm_mul_ps(crsrcrcr.m, cpcpspcp.m), cycycysy.m)
    };
    quat right = {
        .m = _mm_mul_ps(
            _mm_mul_ps(_mm_shuffle_ps(crsrcrcr.m, crsrcrcr.m, _MM_SHUFFLE(1, 1, 0, 1)), _mm_shuffle_ps(cpcpspcp.m, cpcpspcp.m, _MM_SHUFFLE(2, 0, 2, 2))),
            _mm_shuffle_ps(cycycysy.m, cycycysy.m, _MM_SHUFFLE(0, 3, 3, 3))
        )
    };

    return (quat) {
        .m = _mm_add_ps(left.m, _mm_or_ps(etqor.m, right.m))
    };
}
/* Multiplies two quats(q1, q2) with each other returning a new quat. */
quat multiplyQuats(const quat q1, const quat q2) {
    quat w = {
        .m = _mm_mul_ps(_mm_shuffle_ps(q1.m, q1.m, _MM_SHUFFLE(0, 0, 0, 0)), _mm_shuffle_ps(q2.m, q2.m, _MM_SHUFFLE(3, 2, 1, 0)))
    };
    quat x = {
        .m = _mm_mul_ps(_mm_shuffle_ps(q1.m, q1.m, _MM_SHUFFLE(1, 1, 1, 1)), _mm_shuffle_ps(q2.m, q2.m, _MM_SHUFFLE(2, 3, 0, 1)))
    };
    quat y = {
        .m = _mm_mul_ps(_mm_shuffle_ps(q1.m, q1.m, _MM_SHUFFLE(2, 2, 2, 2)), _mm_shuffle_ps(q2.m, q2.m, _MM_SHUFFLE(1, 0, 3, 2)))
    };
    quat z = {
        .m = _mm_mul_ps(_mm_shuffle_ps(q1.m, q1.m, _MM_SHUFFLE(3, 3, 3, 3)), _mm_shuffle_ps(q2.m, q2.m, _MM_SHUFFLE(0, 1, 2, 3)))
    };

    return (quat) {
        .m = _mm_add_ps(_mm_add_ps(_mm_add_ps(w.m, _mm_xor_ps(mqor1.m, x.m)), _mm_xor_ps(mqor2.m, y.m)), _mm_xor_ps(mqor3.m, z.m))
    };
}
/* Creates a matrix from a given quaternion with translation x, y, z. */
mat4x4 MatfromQuat(const quat q, const float x, const float y, const float z) {
    mat4x4 m;
    vec4 r1 = {
        .m = _mm_mul_ps(_mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 1, 1, 0)), _mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 3, 2, 0)))
    };
    vec4 r2 = {
        .m = _mm_mul_ps(_mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 0, 0, 1)), _mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 2, 3, 1)))
    };
    m.m[0].m = _mm_sub_ps(_mm_mul_ps(_mm_add_ps(r1.m, _mm_xor_ps(mfqor1.m, r2.m)), twos.m), ones1.m);

    r1.m = _mm_mul_ps(_mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 2, 0, 1)), _mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 3, 0, 2)));
    r2.m = _mm_mul_ps(_mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 0, 2, 0)), _mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 1, 2, 3)));
    m.m[1].m = _mm_sub_ps(_mm_mul_ps(_mm_add_ps(r1.m, _mm_xor_ps(mfqor2.m, r2.m)), twos.m), ones2.m);

    r1.m = _mm_mul_ps(_mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 0, 2, 1)), _mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 0, 3, 3)));
    r2.m = _mm_mul_ps(_mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 3, 0, 0)), _mm_shuffle_ps(q.m, q.m, _MM_SHUFFLE(0, 3, 1, 2)));
    m.m[2].m = _mm_sub_ps(_mm_mul_ps(_mm_add_ps(r1.m, _mm_xor_ps(mfqor3.m, r2.m)), twos.m), ones3.m);

    vec4 xyz = {
        _mm_setr_ps(x, y, z, 1.f)
    };
    vec4 x1 = {
        _mm_mul_ps(_mm_shuffle_ps(xyz.m, xyz.m, _MM_SHUFFLE(3, 0, 0, 0)), m.m[0].m)
    };
    vec4 y1 = {
        _mm_mul_ps(_mm_shuffle_ps(xyz.m, xyz.m, _MM_SHUFFLE(3, 1, 1, 1)), m.m[1].m)
    };
    vec4 z1 = {
        _mm_mul_ps(_mm_shuffle_ps(xyz.m, xyz.m, _MM_SHUFFLE(3, 2, 2, 2)), m.m[2].m)
    };
    m.m[3].m = _mm_sub_ps(_mm_sub_ps(_mm_sub_ps(xyz.m, x1.m), y1.m), z1.m);

    return m;
}
/* Creates a quaternion from a rotation Matrix. Scale is not taken in account. */
quat quatFromMat(mat4x4 m) {
    float trace = _mm_cvtss_f32(m.m[0].m) +
        _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(0, 3, 2, 1))) +
        _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(1, 0, 3, 2)));

    if (trace > 0) {
        float s = 0.5f / sqrtf(trace + 1.f);
        return (quat) {
            0.25f / s,
            (_mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(0, 3, 2, 1))) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(1, 0, 3, 2)))) * s,
            (_mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(1, 0, 3, 2))) - _mm_cvtss_f32(m.m[2].m)) * s,
            (_mm_cvtss_f32(m.m[1].m) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(0, 3, 2, 1)))) * s
        };

    } else {
        if ((_mm_cvtss_f32(m.m[0].m) > _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(0, 3, 2, 1)))) && (_mm_cvtss_f32(m.m[0].m) > _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(1, 0, 3, 2))))) {
            float s = 2.f * sqrtf(1.f + _mm_cvtss_f32(m.m[0].m) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(0, 3, 2, 1))) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(1, 0, 3, 2))));
            return (quat) {
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(0, 3, 2, 1))) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(1, 0, 3, 2)))) / s,
                0.25f * s,
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(0, 3, 2, 1))) + _mm_cvtss_f32(m.m[1].m)) / s,
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(1, 0, 3, 2))) + _mm_cvtss_f32(m.m[2].m)) / s
            };
        } else if (_mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(0, 3, 2, 1))) > _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(1, 0, 3, 2)))) {
            float s = 2.f * sqrtf(1.f + _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(0, 3, 2, 1))) - _mm_cvtss_f32(m.m[0].m) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(1, 0, 3, 2))));
            return (quat) {
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(1, 0, 3, 2))) - _mm_cvtss_f32(m.m[2].m)) / s,
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(0, 3, 2, 1))) + _mm_cvtss_f32(m.m[1].m)) / s,
                0.25f * s,
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(1, 0, 3, 2))) + _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(0, 3, 2, 1)))) / s
            };
        } else {
            float s = 2.f * sqrtf(1.f + _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(1, 0, 3, 2))) - _mm_cvtss_f32(m.m[0].m) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(0, 3, 2, 1))));
            return (quat) {
                (_mm_cvtss_f32(m.m[1].m) - _mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(0, 3, 2, 1)))) / s,
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[0].m, m.m[0].m, _MM_SHUFFLE(1, 0, 3, 2))) + _mm_cvtss_f32(m.m[2].m)) / s,
                (_mm_cvtss_f32(_mm_shuffle_ps(m.m[1].m, m.m[1].m, _MM_SHUFFLE(1, 0, 3, 2))) + _mm_cvtss_f32(_mm_shuffle_ps(m.m[2].m, m.m[2].m, _MM_SHUFFLE(0, 3, 2, 1)))) / s,
                0.25f * s
            };
        }
    }
}
/* Spherical interpolates between two quaternions at coefficient t. */
quat slerp(const quat q1, const quat q2, const float t) {
    // Calculate angle between q1 and q2.(Dot Product).
    quat r = {
        .m = _mm_mul_ps(q1.m, q2.m)
    };
    const float cosHalfTheta = _mm_cvtss_f32(
        _mm_add_ps(
            _mm_add_ps(r.m, _mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 0, 0, 1))),
            _mm_add_ps(_mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 0, 0, 2)), _mm_shuffle_ps(r.m, r.m, _MM_SHUFFLE(0, 0, 0, 3)))
        )
    );

    // if q1 = q2 or q1 = -q2 then theta = 0 and we can return q1;
    if (fabs(cosHalfTheta) >= 1.f)
        return q1;

    // Calculate temporary values.
    const float sinHalfTheta = sqrtf(1.f - (cosHalfTheta * cosHalfTheta));
    // if theta = 180 degrees then result is not fully defined.We could rotate arround any axis normal to q1 or q2.
    if (fabs(sinHalfTheta) < 0.001f) {
        return (quat) {
            .m = _mm_add_ps(_mm_mul_ps(q1.m, slhalf.m), _mm_mul_ps(q2.m, slhalf.m))
        };
    }

    const float halfTheta = acosf(cosHalfTheta);
    const quat ratioA = {
        .m = _mm_set_ps1(sinf((1.f - t) * halfTheta) / sinHalfTheta)
    };
    const quat ratioB = {
        .m = _mm_set_ps1(sinf(t * halfTheta) / sinHalfTheta)
    };

    return (quat) {
        .m = _mm_add_ps(_mm_mul_ps(q1.m, ratioA.m), _mm_mul_ps(q2.m, ratioB.m))
    };
}
/* Linearly interpolates between two quaternions at coefficient t. */
quat lerp(const quat q1, const quat q2, const float t) {
    const quat scale = {
        .m = _mm_set_ps1(1.f - t)
    };
    const quat tval = {
        .m = _mm_set_ps1(t)
    };
    return (quat) {
        .m = _mm_add_ps(_mm_mul_ps(q1.m, scale.m), _mm_mul_ps(q2.m, tval.m))
    };
}
#else // ITERATIVE_CODE #########################################################################################
/* Creates an initialized unit quaternion. */
quat unitQuat(void) {
    return (quat) { 1.f, 0.f, 0.f, 0.f };
}
/* Sets a quat's parameters, creating a new quat. */
quat setQuat(const float angle, const float x, const float y, const float z) {
    return (quat) { angle, x, y, z };
}
/* Computes the magnitude aka length of a given quat. */
float magnitudeQuat(const quat q) {
    quat r = { q.c[0] * q.c[0], q.c[1] * q.c[1], q.c[2] * q.c[2], q.c[3] * q.c[3] };
    return sqrtf(r.c[0] + r.c[1] + r.c[2] + r.c[3]);
}
/* Normalizes given quat if its not already. */
void normalizeQuat(quat* q) {
    float check = (q->c[0] * q->c[0]) + (q->c[1] * q->c[1]) + (q->c[2] * q->c[2]) + (q->c[3] * q->c[3]);
    if (check > 1.000001f) {
        float magnitude = sqrtf(check);
        q->c[0] /= magnitude;
        q->c[1] /= magnitude;
        q->c[2] /= magnitude;
        q->c[3] /= magnitude;
    }
}
/* Conjugate quat's vector part, aka( negating the sign ). */
quat conjugateQuat(const quat q) {
    return (quat) {
        q.c[0],
            -q.c[1],
            -q.c[2],
            -q.c[3]
    };
}
/* Creates a rotation quaternion with angle W and rotation axis X Y Z: */
quat rotationQuat(const float angle, const float x, const float y, const float z) {
    const float radius = (angle * (3.14159f / 180.0f)) * 0.5f;
    const float sn = sinf(radius);
    return (quat) {
        cosf(radius),
            x* sn,
            y* sn,
            z* sn
    };
}
/* Rotates vector v by the given quaternion.Returns a new vector. */
vec4 vec4RotateQuat(const quat q, const vec4 v) {
    quat r = setQuat(0.f, v.c[0], v.c[1], v.c[2]);
    r = multiplyQuats(multiplyQuats(conjugateQuat(q), r), q);
    return (vec4) { r.c[1], r.c[2], r.c[3], v.c[3] };
}
/* Rotates vector v by the given quaternion. */
void setvec4RotateQuat(const quat q, vec4* v) {
    quat r = setQuat(0.f, v->c[0], v->c[1], v->c[2]);
    r = multiplyQuats(multiplyQuats(conjugateQuat(q), r), q);
    v->c[0] = r.c[1];
    v->c[1] = r.c[2];
    v->c[2] = r.c[3];
}
/* Adds quats q1 and q2 together returning a new quat. */
quat addQuats(const quat q1, const quat q2) {
    return (quat) {
        q1.c[0] + q2.c[0],
        q1.c[1] + q2.c[1],
        q1.c[2] + q2.c[2],
        q1.c[3] + q2.c[3]
    };
}
/* Creates a quaternion from the given euler angles. */
quat eulertoQuat(const float roll, const float yaw, const float pitch) { 
    const float half_roll = roll * 0.5f;
    const float half_yaw = yaw * 0.5f;
    const float half_pitch = pitch * 0.5f;

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cpcy = cp * cy;
    const float spsy = sp * sy;
    const float spcy = sp * cy;
    const float cpsy = cp * sy;

    return (quat) {
        (cr * cpcy) + (sr * spsy),
        (sr * cpcy) - (cr * spsy),
        (cr * spcy) + (sr * cpsy),
        (cr * cpsy) - (sr * spcy),
    };
}
/* Multiplies two quats(q1, q2) with each other returning a new quat. */
quat multiplyQuats(const quat q1, const quat q2) {
    return (quat) {
        (q1.c[0] * q2.c[0]) - (q1.c[1] * q2.c[1]) - (q1.c[2] * q2.c[2]) - (q1.c[3] * q2.c[3]),
        (q1.c[0] * q2.c[1]) + (q1.c[1] * q2.c[0]) + (q1.c[2] * q2.c[3]) - (q1.c[3] * q2.c[2]),
        (q1.c[0] * q2.c[2]) - (q1.c[1] * q2.c[3]) + (q1.c[2] * q2.c[0]) + (q1.c[3] * q2.c[1]),
        (q1.c[0] * q2.c[3]) + (q1.c[1] * q2.c[2]) - (q1.c[2] * q2.c[1]) + (q1.c[3] * q2.c[0])
    };
}
/* Creates a matrix from a given quaternion with translation x, y, z. */
mat4x4 MatfromQuat(const quat q, const float x, const float y, const float z) {
    mat4x4 m;
    m.m[0].c[0] = 2.0f * ((q.c[0] * q.c[0]) + (q.c[1] * q.c[1])) - 1.0f;
    m.m[0].c[1] = 2.0f * ((q.c[1] * q.c[2]) - (q.c[0] * q.c[3]));
    m.m[0].c[2] = 2.0f * ((q.c[1] * q.c[3]) + (q.c[0] * q.c[2]));
    m.m[0].c[3] = 0.0f;

    m.m[1].c[0] = 2.0f * ((q.c[1] * q.c[2]) + (q.c[0] * q.c[3]));
    m.m[1].c[1] = 2.0f * ((q.c[0] * q.c[0]) + (q.c[2] * q.c[2])) - 1.0f;
    m.m[1].c[2] = 2.0f * ((q.c[2] * q.c[3]) - (q.c[0] * q.c[1]));
    m.m[1].c[3] = 0.0f;

    m.m[2].c[0] = 2.0f * ((q.c[1] * q.c[3]) - (q.c[0] * q.c[2]));
    m.m[2].c[1] = 2.0f * ((q.c[2] * q.c[3]) + (q.c[0] * q.c[1]));
    m.m[2].c[2] = 2.0f * ((q.c[0] * q.c[0]) + (q.c[3] * q.c[3])) - 1.0f;
    m.m[2].c[3] = 0.0f;

    if (m.m[2].c[0] != 0)
        m.m[3].c[0] = x - x * m.m[0].c[0] - y * m.m[1].c[0] - z * m.m[2].c[0];
    if (m.m[2].c[1] != 0)
        m.m[3].c[1] = y - x * m.m[0].c[1] - y * m.m[1].c[1] - z * m.m[2].c[1];
    if (m.m[2].c[2] != 0)
        m.m[3].c[2] = z - x * m.m[0].c[2] - y * m.m[1].c[2] - z * m.m[2].c[2];
    m.m[3].c[3] = 1.0;

    return m;
}
/* Creates a quaternion from a rotation Matrix. Scale is not taken in account. */
quat quatFromMat(mat4x4 m) {
    float trace = m.m[0].c[0] + m.m[1].c[1] + m.m[2].c[2];
    if (trace > 0) {
        float s = 0.5f / sqrtf(trace + 1.f);
        return (quat) {
            0.25f / s,
            (m.m[2].c[1] - m.m[1].c[2]) * s,
            (m.m[0].c[2] - m.m[2].c[0]) * s,
            (m.m[1].c[0] - m.m[0].c[1]) * s
        };
    } else {
        if ((m.m[0].c[0] > m.m[1].c[1]) && (m.m[0].c[0]) > m.m[2].c[2]) {
            float s = 2.f * sqrtf(1.f + m.m[0].c[0] - m.m[1].c[1] - m.m[2].c[2]);
            return (quat) {
                (m.m[2].c[1] - m.m[1].c[2]) / s,
                0.25f * s,
                (m.m[0].c[1] + m.m[1].c[0]) / s,
                (m.m[0].c[2] + m.m[2].c[0]) / s
            };
        } else if (m.m[1].c[1] > m.m[2].c[2]) {
            float s = 2.f * sqrtf(1.f + m.m[1].c[1] - m.m[0].c[0] - m.m[2].c[2]);
            return (quat) {
                (m.m[0].c[2] - m.m[2].c[0]) / s,
                (m.m[0].c[1] + m.m[1].c[0]) / s,
                0.25f * s,
                (m.m[1].c[2] + m.m[2].c[1]) / s
            };
        } else {
            float s = 2.f * sqrtf(1.f + m.m[2].c[2] - m.m[0].c[0] - m.m[1].c[1]);
            return (quat) {
                (m.m[1].c[0] - m.m[0].c[1]) / s,
                (m.m[0].c[2] + m.m[2].c[0]) / s,
                (m.m[1].c[2] + m.m[2].c[1]) / s,
                0.25f * s
            };
        }
    }
}
/* Spherical interpolates between two quaternions at coefficient t. */
quat slerp(const quat q1, const quat q2, const float t) {
    // Calculate angle between q1 and q2.
    const float cosHalfTheta = (q1.c[0] * q2.c[0]) + (q1.c[1] * q2.c[1]) + (q1.c[2] * q2.c[2]) + (q1.c[3] * q2.c[3]);
    // if q1 = q2 or q1 = -q2 then theta = 0 and we can return q1;
    if (fabs(cosHalfTheta) >= 1.f)
        return q1;

    // Calculate temporary values.
    const float sinHalfTheta = sqrtf(1.f - (cosHalfTheta * cosHalfTheta));
    // if theta = 180 degrees then result is not fully defined.We could rotate arround any axis normal to q1 or q2.
    if (fabs(sinHalfTheta) < 0.001f) {
        return (quat) {
            (q1.c[0] * 0.5f) + (q2.c[0] * 0.5f),
            (q1.c[1] * 0.5f) + (q2.c[1] * 0.5f),
            (q1.c[2] * 0.5f) + (q2.c[2] * 0.5f),
            (q1.c[3] * 0.5f) + (q2.c[3] * 0.5f)
        };
    }

    const float halfTheta = acosf(cosHalfTheta);
    const float ratioA = sinf((1.f - t) * halfTheta) / sinHalfTheta;
    const float ratioB = sinf(t * halfTheta) / sinHalfTheta;
    // Calculate the quaternion.
    return (quat) {
        (q1.c[0] * ratioA) + (q2.c[0] * ratioB),
        (q1.c[1] * ratioA) + (q2.c[1] * ratioB),
        (q1.c[2] * ratioA) + (q2.c[2] * ratioB),
        (q1.c[3] * ratioA) + (q2.c[3] * ratioB)
    };
}
/* Linearly interpolates between two quaternions at coefficient t. */
quat lerp(const quat q1, const quat q2, const float t) {
    const float scale = 1.f - t;
    return (quat) {
        (q1.c[0] * scale) + (q2.c[0] * t),
        (q1.c[1] * scale) + (q2.c[1] * t),
        (q1.c[2] * scale) + (q2.c[2] * t),
        (q1.c[3] * scale) + (q2.c[3] * t)
    };
}
#endif // VECTORIZED_CODE #######################################################################################

#endif QMATH_IMPLEMENTATION

#endif // QMATH_H