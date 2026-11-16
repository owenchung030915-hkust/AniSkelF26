#ifndef QUATERNION_H
#define QUATERNION_H

#include "vec.h"
#include <cmath>

// Unit quaternion. Used by the camera so interactive tumbling
// does not pass through the Euler-angle singularity (gimbal lock).
struct Quat
{
	float w, x, y, z;

	Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
	Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

	static Quat axisAngle(Vec3f axis, float radians)
	{
		float len = (float)axis.length();
		if (len < 1e-8f)
			return Quat();
		axis /= len;
		float h = radians * 0.5f;
		float s = sinf(h);
		return Quat(cosf(h), axis[0] * s, axis[1] * s, axis[2] * s);
	}

	Quat operator*(const Quat& b) const
	{
		return Quat(
			w * b.w - x * b.x - y * b.y - z * b.z,
			w * b.x + x * b.w + y * b.z - z * b.y,
			w * b.y - x * b.z + y * b.w + z * b.x,
			w * b.z + x * b.y - y * b.x + z * b.w);
	}

	Vec3f rotate(const Vec3f& v) const
	{
		Quat qv(0.0f, v[0], v[1], v[2]);
		Quat inv(w, -x, -y, -z);
		Quat r = (*this) * qv * inv;
		return Vec3f(r.x, r.y, r.z);
	}

	void normalize()
	{
		float n = sqrtf(w * w + x * x + y * y + z * z);
		if (n > 1e-8f) {
			w /= n; x /= n; y /= n; z /= n;
		}
	}

	// Columns are the images of +X, +Y and +Z.
	static Quat fromBasis(const Vec3f& right, const Vec3f& up, const Vec3f& localZ)
	{
		float m00 = right[0], m01 = up[0], m02 = localZ[0];
		float m10 = right[1], m11 = up[1], m12 = localZ[1];
		float m20 = right[2], m21 = up[2], m22 = localZ[2];
		float tr = m00 + m11 + m22;
		Quat q;
		if (tr > 0.0f) {
			float s = sqrtf(tr + 1.0f) * 2.0f;
			q.w = 0.25f * s;
			q.x = (m21 - m12) / s;
			q.y = (m02 - m20) / s;
			q.z = (m10 - m01) / s;
		}
		else if (m00 > m11 && m00 > m22) {
			float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
			q.w = (m21 - m12) / s;
			q.x = 0.25f * s;
			q.y = (m01 + m10) / s;
			q.z = (m02 + m20) / s;
		}
		else if (m11 > m22) {
			float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
			q.w = (m02 - m20) / s;
			q.x = (m01 + m10) / s;
			q.y = 0.25f * s;
			q.z = (m12 + m21) / s;
		}
		else {
			float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
			q.w = (m10 - m01) / s;
			q.x = (m02 + m20) / s;
			q.y = (m12 + m21) / s;
			q.z = 0.25f * s;
		}
		q.normalize();
		return q;
	}
};

#endif
