#ifndef GEOM_UTIL_H
#define GEOM_UTIL_H

#include "Primitives.h"
#include "Vec3.h"

namespace Positional
{
	static class GeomUtil
	{
	private:
		GeomUtil() = delete;
		static bool raycastCaps(const Vec3 &c0, const Vec3 &c1, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance);

	public:
		inline static Vec3 nearestOnSegment(const Vec3& point, const Vec3& a0, const Vec3& a1)
		{
			const Vec3 u = a1 - a0;
			const Vec3 v = point - a0;
			const Float t = Math::clamp(v.dot(u) / u.lengthSq(), 0, 1);
			return a0 + u * t;
		}

		inline static Vec3 nearestOnRay(const Vec3 &point, const Vec3 &r0, const Vec3 &n)
		{
			return r0 + n * Math::max((point - r0).dot(n), 0);
		}

		static Vec3 nearestOnTriangle(const Vec3 &point, const Vec3 &a, const Vec3 &b, const Vec3 &c);
		static Vec3 nearestOnTetraderon(const Vec3 &point, const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d);

		static void nearestOnSegments(const Vec3 &a0, const Vec3 &a1, const Vec3 &b0, const Vec3 &b1, Vec3 &outA, Vec3 &outB);
		static void nearestOnRaySegment(const Vec3 &r0, const Vec3 &n, const Vec3 &c0, const Vec3 &c1, Vec3 &outA, Vec3 &outB);
		
		static bool raycastBox(const Vec3 &extents, const Vec3 &r0, const Vec3 &n, const Vec3 &ni, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance);
		static bool raycastSphere(const Vec3 &center, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance);
		static bool raycastCapsule(const Vec3 &c0, const Vec3 &c1, const Float &length, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance);
		static bool raycastTriangle(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &r0, const Vec3 &rn, const Float &maxDist, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance);

		/*
		* Raycast cylinder without end faces
		*/
		static bool raycastCylinder(const Vec3 &c0, const Vec3 &c1, const Float &length, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance);
	};
}

#endif // GEOM_UTIL_H