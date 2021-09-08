/*
 * A capsule collider. The capsule length is always along the y-axis.
 * The rotation property can but used for other orientations.
 */
#ifndef CAPSULE_COLLIDER_H
#define CAPSULE_COLLIDER_H

#include "ShapeId.h"
#include "mass/Volume.h"

namespace Positional::Collision
{
	struct CapsuleCollider final
	{
		static UInt8 shapeId() { return ShapeId::Capsule; }

		static Bounds bounds(const Collider &collider)
		{
			const Float l_2 = collider.shape.length * 0.5;
			const Vec3 c0 = collider.pointToWorld(Vec3(0, -l_2, 0));
			const Vec3 c1 = collider.pointToWorld(Vec3(0, l_2, 0));

			Bounds bounds = Bounds(c0, Vec3(collider.shape.radius));
			bounds.merge(Bounds(c1, Vec3(collider.shape.radius)));
			return bounds;
		}

		static Float volume(const Collider &collider) { return Mass::capsuleVolume(collider.shape.radius, collider.shape.length); }

		static bool raycast(const Collider &collider, const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
		{
			const Float l_2 = collider.shape.length * 0.5;
			const Vec3 c0 = collider.pointToWorld(Vec3(0, -l_2, 0));
			const Vec3 c1 = collider.pointToWorld(Vec3(0, l_2, 0));
			return GeomUtil::raycastCapsule(c0, c1, collider.shape.length, collider.shape.radius, ray.origin, ray.normal(), maxDistance, outPoint, outNormal, outDistance);
		}

		static Vec3 localSupport(const Collider &collider, const Vec3 &axis)
		{
			const Float l_2 = collider.shape.length * 0.5;
			const Vec3 c[2] = {Vec3(0, -l_2, 0), Vec3(0, l_2, 0)};

			const Float dot0 = axis.dot(c[0]);
			const Float dot1 = axis.dot(c[1]);
			
			if (Math::approx(dot0, dot1))
			{
				return axis * collider.shape.radius;
			}

			return c[dot1 > dot0] + axis * collider.shape.radius;
		}
	private:
		CapsuleCollider() {}
	};
}
#endif // CAPSULE_COLLIDER_H