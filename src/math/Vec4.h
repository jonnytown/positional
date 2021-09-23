#include "Primitives.h"

#ifndef VEC4_H
#define VEC4_H

#include "Vec3.h"

/* 
 * A vector class with 4 components
 */
namespace Positional
{
	class Vec4
	{
	public:
		Float x, y, z, w;

		Vec4() : x(0), y(0), z(0), w(0) {}

		Vec4(const Float &_x, const Float &_y, const Float &_z, const Float &_w)
			: x(_x), y(_y), z(_z), w(_w) {}

		Vec4(const Vec3 &vec3, const Float &_w)
			: x(vec3.x), y(vec3.y), z(vec3.z), w(_w) {}

		Vec4(const Float &v) : x(v), y(v), z(v), w(v) {}

		Vec4 &operator=(const Vec4 &rhs)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			w = rhs.w;
			return *this;
		}

		inline bool operator==(const Vec4 &rhs) const
		{
			return Math::approx(x, rhs.x) && Math::approx(y, rhs.y) && Math::approx(z, rhs.z) && Math::approx(w, rhs.w);
		}

		inline bool operator!=(const Vec4 &rhs) const
		{
			return !Math::approx(x, rhs.x) || !Math::approx(y, rhs.y) || !Math::approx(z, rhs.z) || !Math::approx(w, rhs.w);
		}

		inline Float &operator[](const UInt8 &axis)
		{
			assert(axis < 4);
			return *(reinterpret_cast<Float *>(this) + axis);
		}

		inline Vec4 operator-() const
		{
			return Vec4(-x, -y, -z, -w);
		}

		inline Vec4 operator+(const Vec4 &rhs) const
		{
			return Vec4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
		}

		inline Vec4 operator-(const Vec4 &rhs) const
		{
			return Vec4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
		}

		inline Vec4 operator*(const Vec4 &rhs) const
		{
			return Vec4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w);
		}

		inline Vec4 operator/(const Vec4 &rhs) const
		{
			return Vec4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w);
		}

		inline Vec4 operator+(const Float &rhs) const
		{
			return Vec4(x + rhs, y + rhs, z + rhs, w + rhs);
		}

		inline friend Vec4 operator+(Float lhs, const Vec4 &rhs)
		{
			return Vec4(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z, lhs + rhs.w);
		}

		inline Vec4 operator-(const Float &rhs) const
		{
			return Vec4(x - rhs, y - rhs, z - rhs, w - rhs);
		}

		inline friend Vec4 operator-(Float lhs, const Vec4 &rhs)
		{
			return Vec4(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z, lhs - rhs.w);
		}

		inline Vec4 operator*(const Float &rhs) const
		{
			return Vec4(x * rhs, y * rhs, z * rhs, w * rhs);
		}

		inline friend Vec4 operator*(Float lhs, const Vec4 &rhs)
		{
			return Vec4(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
		}

		inline Vec4 operator/(const Float &rhs) const
		{
			return Vec4(x / rhs, y / rhs, z / rhs, w / rhs);
		}

		inline friend Vec4 operator/(Float lhs, const Vec4 &rhs)
		{
			return Vec4(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w);
		}

		inline Float length() const
		{
			return Math::sqrt(x * x + y * y + z * z + w * w);
		}

		inline Float lengthSq() const
		{
			return x * x + y * y + z * z + w * w;
		}

		inline Vec4 normalized() const
		{
			Float invNorm = 1.0 / Math::sqrt(x * x + y * y + z * z + w * w);
			return Vec4(x * invNorm, y * invNorm, z * invNorm, w * invNorm);
		}

		static const Vec4 zero;
		static const Vec4 one;
	};
}

#endif // VEC4_H