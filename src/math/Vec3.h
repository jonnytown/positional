/* 
 * A vectors class with three components
 */
#include "Primitives.h"
#include "Util.h"
#include <assert.h>

#ifndef VEC3_H
#define VEC3_H

namespace Positional
{
	class Vec3
	{
	public:
		Float x, y, z;

		Vec3() : x(0), y(0), z(0) {}
		Vec3(const Float &_x, const Float &_y, const Float &_z) : x(_x), y(_y), z(_z) {}
		Vec3(const Float &v) : x(v), y(v), z(v) {}

		Vec3 &operator=(const Vec3 &rhs)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			return *this;
		}

		inline Float &operator[](const UInt8 &axis)
		{
			assert(axis < 3);
			return *(reinterpret_cast<Float *>(this) + axis);
		}

		inline bool operator==(const Vec3 &rhs) const
		{
			return Math::approx(x, rhs.x) && Math::approx(y, rhs.y) && Math::approx(z, rhs.z);
		}

		inline bool operator!=(const Vec3 &rhs) const
		{
			return !Math::approx(x, rhs.x) || !Math::approx(y, rhs.y) || !Math::approx(z, rhs.z);
		}

		inline Vec3 operator-() const
		{
			return Vec3(-x, -y, -z);
		}

		inline void operator+=(const Vec3 &rhs)
		{
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
		}

		inline void operator-=(const Vec3 &rhs)
		{
			x -= rhs.x;
			y -= rhs.y;
			z -= rhs.z;
		}

		inline Vec3 operator+(const Vec3 &rhs) const
		{
			return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
		}

		inline Vec3 operator-(const Vec3 &rhs) const
		{
			return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
		}

		inline Vec3 operator*(const Vec3 &rhs) const
		{
			return Vec3(x * rhs.x, y * rhs.y, z * rhs.z);
		}

		inline Vec3 operator/(const Vec3 &rhs) const
		{
			return Vec3(x / rhs.x, y / rhs.y, z / rhs.z);
		}

		inline Vec3 operator+(const Float &rhs) const
		{
			return Vec3(x + rhs, y + rhs, z + rhs);
		}

		inline friend Vec3 operator+(Float lhs, const Vec3 &rhs)
		{
			return Vec3(lhs + rhs.x, lhs + rhs.y, lhs + rhs.z);
		}

		inline Vec3 operator-(const Float &rhs) const
		{
			return Vec3(x - rhs, y - rhs, z - rhs);
		}

		inline friend Vec3 operator-(Float lhs, const Vec3 &rhs)
		{
			return Vec3(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
		}

		inline Vec3 operator*(const Float &rhs) const
		{
			return Vec3(x * rhs, y * rhs, z * rhs);
		}

		inline friend Vec3 operator*(Float lhs, const Vec3 &rhs)
		{
			return Vec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
		}

		inline Vec3 operator/(const Float &rhs) const
		{
			return Vec3(x / rhs, y / rhs, z / rhs);
		}

		inline friend Vec3 operator/(Float lhs, const Vec3 &rhs)
		{
			return Vec3(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
		}

		inline bool approx(const Vec3 &rhs, const Float &epsilon = Math::Epsilon)
		{
			return Math::approx(x, rhs.x, epsilon) && Math::approx(y, rhs.y, epsilon) && Math::approx(z, rhs.z, epsilon);
		}

		inline Float length() const
		{
			return Math::sqrt(x * x + y * y + z * z);
		}

		inline Float lengthSq() const
		{
			return x * x + y * y + z * z;
		}

		inline Vec3 &normalize()
		{
			Float invNorm = 1.0 / Math::sqrt(x * x + y * y + z * z);
			x *= invNorm;
			y *= invNorm;
			z *= invNorm;;
			return *this;
		}

		inline Vec3 normalized() const
		{
			Float invNorm = 1.0 / Math::sqrt(x * x + y * y + z * z);
			return Vec3(x * invNorm, y * invNorm, z * invNorm);
		}

		inline Vec3 abs() const
		{
			return Vec3(Math::abs(x), Math::abs(y), Math::abs(z));
		}

		inline Float distance(const Vec3 &other) const
		{
			Vec3 result(other.x - x, other.y - y, other.z - z);
			return result.length();
		}

		inline Float distanceSq(const Vec3 &other) const
		{
			Vec3 result(other.x - x, other.y - y, other.z - z);
			return result.lengthSq();
		}

		inline Float dot(const Vec3 &other) const
		{
			return x * other.x + y * other.y + z * other.z;
		}

		inline Vec3 cross(const Vec3 &other) const
		{
			return Vec3(
				y * other.z - z * other.y,
				z * other.x - x * other.z,
				x * other.y - y * other.x);
		}

		inline Vec3 reflect(const Vec3 &normal) const
		{
			Float dot2 = (x * normal.x + y * normal.y + z * normal.z)*2.0;
			return Vec3(x - normal.x * dot2, y - normal.y * dot2, z - normal.z * dot2);
		}

		/*
		 * Project onto other vector
		 */
		inline Vec3 project(const Vec3 &other) const
		{
			return other * (this->dot(other)/other.dot(other));
		}

		/*
		 * Project onto the plane defined by normal
		 */
		inline Vec3 projectOnPlane(const Vec3 &normal) const
		{
			return *this - normal * (this->dot(normal) / normal.dot(normal));
		}

		static const Vec3 zero;
		static const Vec3 one;
		static const Vec3 neg_x;
		static const Vec3 pos_x;
		static const Vec3 neg_y;
		static const Vec3 pos_y;
		static const Vec3 neg_z;
		static const Vec3 pos_z;
	};
}

#endif // VEC3_H