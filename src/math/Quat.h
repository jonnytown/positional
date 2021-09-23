/* 
 * Quaternion class
 */

#include "Primitives.h"
#include "Vec3.h"
#include "Vec4.h"

#ifndef QUAT_H
#define QUAT_H

namespace Positional
{
	class Quat
	{
	public:
		Float x, y, z, w;

		Quat() : x(0.0), y(0.0), z(0.0), w(1.0) {}

		Quat(const Float &v) : x(v), y(v), z(v), w(v) {}

		Quat(const Float &_x, const Float &_y, const Float &_z, const Float &_w)
			: x(_x), y(_y), z(_z), w(_w) {}

		Quat& operator=(const Quat &rhs)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			w = rhs.w;
			return *this;
		}

		inline bool operator==(const Quat &rhs) const
		{
			return Math::approx(x, rhs.x) && Math::approx(y, rhs.y) && Math::approx(z, rhs.z) && Math::approx(w, rhs.w);
		}

		inline bool operator!=(const Quat &rhs) const
		{
			return !Math::approx(x, rhs.x) || !Math::approx(y, rhs.y) || !Math::approx(z, rhs.z) || !Math::approx(w, rhs.w);
		}

		inline Float &operator[](const UInt8 &axis)
		{
			assert(axis < 4);
			return *(reinterpret_cast<Float *>(this) + axis);
		}

		inline bool isIdentity() const
		{
			return x == 0.0 && y == 0.0 && z == 0.0 && w == 1.0;
		}

		inline Quat &normalize()
		{
			Float invNorm = 1.0 / Math::sqrt(x * x + y * y + z * z + w * w);
			x *= invNorm;
			y *= invNorm;
			z *= invNorm;
			w *= invNorm;
			return *this;
		}

		inline Quat normalized() const
		{
			Float invNorm = 1.0 / Math::sqrt(x * x + y * y + z * z + w * w);
			return Quat(x * invNorm, y * invNorm, z * invNorm, w * invNorm);
		}

		inline Quat conjugate() const
		{
			return Quat(-x, -y, -z, w);
		}

		inline Quat inverse() const
		{
			//  -1   (       a              -v       )
			// q   = ( -------------   ------------- )
			//       (  a^2 + |v|^2  ,  a^2 + |v|^2  )
			Float invNorm = 1.0 / (x * x + y * y + z * z + w * w);
			return Quat(-x * invNorm, -y * invNorm, -z * invNorm, w * invNorm);
		}

		inline Float dot(const Quat &rhs) {
			return x * rhs.x + y * rhs.y + z * rhs.z + w *rhs.w;
		}

		inline Quat operator*(const Quat &rhs) const
		{
			return Quat(
				w * rhs.x + rhs.w * x + y * rhs.z - rhs.y * z,
				w * rhs.y + rhs.w * y + z * rhs.x - rhs.z * x,
				w * rhs.z + rhs.w * z + x * rhs.y - rhs.x * y,
				w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z);
		}

		Quat operator/(const Quat &rhs) const
		{
			const Quat inv = rhs.inverse();
			return Quat(
				w * inv.x + inv.w * x + y * inv.z - inv.y * z,
				w * inv.y + inv.w * y + z * inv.x - inv.z * x,
				w * inv.z + inv.w * z + x * inv.y - inv.x * y,
				w * inv.w - x * inv.x - y * inv.y - z * inv.z);
		}

		Vec3 operator*(const Vec3 &rhs) const
		{
			Float x2 = x + x;
			Float y2 = y + y;
			Float z2 = z + z;

			Float wx2 = w * x2;
			Float wy2 = w * y2;
			Float wz2 = w * z2;
			Float xx2 = x * x2;
			Float xy2 = x * y2;
			Float xz2 = x * z2;
			Float yy2 = y * y2;
			Float yz2 = y * z2;
			Float zz2 = z * z2;

			return Vec3(
				rhs.x * (1.0 - yy2 - zz2) + rhs.y * (xy2 - wz2) + rhs.z * (xz2 + wy2),
				rhs.x * (xy2 + wz2) + rhs.y * (1.0f - xx2 - zz2) + rhs.z * (yz2 - wx2),
				rhs.x * (xz2 - wy2) + rhs.y * (yz2 + wx2) + rhs.z * (1.0 - xx2 - yy2)
			);
		}

		Vec4 operator*(const Vec4 &rhs) const
		{
			Float x2 = x + x;
			Float y2 = y + y;
			Float z2 = z + z;

			Float wx2 = w * x2;
			Float wy2 = w * y2;
			Float wz2 = w * z2;
			Float xx2 = x * x2;
			Float xy2 = x * y2;
			Float xz2 = x * z2;
			Float yy2 = y * y2;
			Float yz2 = y * z2;
			Float zz2 = z * z2;

			return Vec4(
				rhs.x * (1.0 - yy2 - zz2) + rhs.y * (xy2 - wz2) + rhs.z * (xz2 + wy2),
				rhs.x * (xy2 + wz2) + rhs.y * (1.0 - xx2 - zz2) + rhs.z * (yz2 - wx2),
				rhs.x * (xz2 - wy2) + rhs.y * (yz2 + wx2) + rhs.z * (1.0 - xx2 - yy2),
				rhs.w
			);
		}

		static inline Quat fromAngleAxis(const Float &angle, const Vec3 &axis)
		{
			Float half = angle * 0.5;
			Float s = Math::sin(half);
			Float c = Math::cos(half);

			return Quat(axis.x*s, axis.y*s, axis.z*s, c);
		}

		static const Quat identity;
	};
}

#endif // VEC4_H