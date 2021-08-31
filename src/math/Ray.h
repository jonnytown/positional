/* 
 * A ray class
 */
#include "Primitives.h"
#include "Vec3.h"

#ifndef RAY_H
#define RAY_H

namespace Positional
{
	class Ray
	{
	public:
		Vec3 origin;
	private:
		Vec3 m_normal, m_invNormal;
	public:
		Ray(const Vec3 &_normal) : origin(Vec3::zero), m_normal(_normal), m_invNormal(1.0/_normal) {}
		Ray(const Vec3 &_origin, const Vec3 &_normal) : origin(_origin), m_normal(_normal), m_invNormal(1.0 / _normal) {}

		const Vec3 &normal() const
		{
			return m_normal;
		}

		const Vec3 &invNormal() const
		{
			return m_invNormal;
		}
	};
}

#endif // RAY_H