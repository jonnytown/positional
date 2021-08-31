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

		bool operator==(const Quat &rhs) const
		{
			return Math::approx(x, rhs.x) && Math::approx(y, rhs.y) && Math::approx(z, rhs.z) && Math::approx(w, rhs.w);
		}

		bool operator!=(const Quat &rhs) const
		{
			return !Math::approx(x, rhs.x) || !Math::approx(y, rhs.y) || !Math::approx(z, rhs.z) || !Math::approx(w, rhs.w);
		}

		inline Float &operator[](const UInt8 &axis)
		{
			assert(axis < 4);
			return ((Float *)this)[axis];
		}

		bool isIdentity() const
		{
			return x == 0.0 && y == 0.0 && z == 0.0 && w == 1.0;
		}

		Quat normalized() const
		{
			Float invNorm = 1.0 / Math::sqrt(x * x + y * y + z * z + w * w);
			return Quat(x * invNorm, y * invNorm, z * invNorm, w * invNorm);
		}

		Quat conjugate() const
		{
			return Quat(-x, -y, -z, w);
		}

		Quat inverse() const
		{
			//  -1   (       a              -v       )
			// q   = ( -------------   ------------- )
			//       (  a^2 + |v|^2  ,  a^2 + |v|^2  )
			Float invNorm = 1.0 / (x * x + y * y + z * z + w * w);
			return Quat(-x * invNorm, -y * invNorm, -z * invNorm, w * invNorm);
		}

		Float dot(const Quat &rhs) {
			return x * rhs.x + y * rhs.y + z * rhs.z + w *rhs.w;
		}

		Quat operator*(const Quat &rhs) const
		{
			Float cx = y * rhs.z - z * rhs.y;
			Float cy = z * rhs.x - x * rhs.z;
			Float cz = x * rhs.y - y * rhs.x;

			Float dot = x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;

			return Quat(
				x * rhs.w + rhs.x * w + cx,
				y * rhs.w + rhs.y * w + cy,
				z * rhs.w + rhs.z * w + cz,
				w * rhs.w - dot
			);
		}

		Quat operator/(const Quat &rhs) const
		{
			const Float invNorm = 1.0 / (x * x + y * y + z * z + w * w);
			const Float rhsX = -x * invNorm;
			const Float rhsY = -y * invNorm;
			const Float rhsZ = -z * invNorm;
			const Float rhsW = w * invNorm;

			const Float cx = y * rhsZ - z * rhsY;
			const Float cy = z * rhsX - x * rhsZ;
			const Float cz = x * rhsY - y * rhsX;

			const Float dot = x * rhsX + y * rhsY + z * rhsZ + w * rhsW;

			return Quat(
				x * rhsW + rhsX * w + cx,
				y * rhsW + rhsY * w + cy,
				z * rhsW + rhsZ * w + cz,
				w * rhsW - dot);
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