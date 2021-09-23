/*
 * A sphere collider
 */
#ifndef SPHERE_COLLIDER_H
#define SPHERE_COLLIDER_H

#include "ShapeId.h"
#include "mass/Volume.h"
#include "mass/Computer.h"

namespace Positional::Collision
{
	struct SphereCollider final
	{
		static UInt8 shapeId() { return ShapeId::Sphere; }

		static Bounds bounds(const Collider &collider)
		{
			return Bounds(collider.pointToWorld(Vec3::zero), Vec3(collider.shape.radius));
		}

		static Float volume(const Collider &collider) { return Mass::sphereVolume(collider.shape.radius); }

		static bool raycast(const Collider &collider, const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
		{
			const Vec3 centerT = collider.pointToWorld(Vec3::zero);
			return GeomUtil::raycastSphere(centerT, collider.shape.radius, ray.origin, ray.normal(), maxDistance, outPoint, outNormal, outDistance);
		}

		static Vec3 localSupport(const Collider &collider, const Vec3 &axis)
		{
			return axis * collider.shape.radius;
		}

		static void computeMass(const Collider &collider, Mass::Computer &computer)
		{
			computer.setSphere(collider.shape.radius, collider.pose.position, collider.density);
		}

		static bool hasRotation() { return false; }

	private:
		SphereCollider() = delete;
	};

	
}
#endif // SPHERE_COLLIDER_H