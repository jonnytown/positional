/*
 * A box collider
 */
#ifndef BOX_COLLIDER_H
#define BOX_COLLIDER_H

#include "ShapeId.h"
#include "mass/Volume.h"
#include "mass/Computer.h"
#include "Collider.h"

namespace Positional::Collision
{
	struct BoxCollider final
	{
		static UInt8 shapeId() { return ShapeId::Box; }

		static Bounds bounds(const Collider &collider) {
			const Vec3 &extents = collider.shape.extents;
			Bounds bounds(collider.pointToWorld(extents), Vec3::zero);
			bounds.merge(collider.pointToWorld(Vec3(extents.x, -extents.y, extents.z)));
			bounds.merge(collider.pointToWorld(Vec3(extents.x, -extents.y, -extents.z)));
			bounds.merge(collider.pointToWorld(Vec3(extents.x, extents.y, -extents.z)));

			bounds.merge(collider.pointToWorld(Vec3(-extents.x, extents.y, extents.z)));
			bounds.merge(collider.pointToWorld(Vec3(-extents.x, -extents.y, extents.z)));
			bounds.merge(collider.pointToWorld(-extents));
			bounds.merge(collider.pointToWorld(Vec3(-extents.x, extents.y, -extents.z)));
			return bounds;
		}

		static Float volume(const Collider &collider) { return Mass::boxVolume(collider.shape.extents); }

		static bool raycast(const Collider &collider, const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
		{
			Ray rayT(collider.pointToLocal(ray.origin), collider.vectorToLocal(ray.normal()));
			if (GeomUtil::raycastBox(collider.shape.extents, rayT.origin, rayT.normal(), rayT.invNormal(), maxDistance, outPoint, outNormal, outDistance))
			{
				outPoint = collider.pointToWorld(outPoint);
				outNormal = collider.vectorToWorld(outNormal);
				return true;
			}
			return false;
		}

		static Vec3 localSupport(const Collider &collider, const Vec3 &axis)
		{
			const Vec3& extents = collider.shape.extents;
			return Vec3(
				Math::sign(axis.x) * extents.x,
				Math::sign(axis.y) * extents.y,
				Math::sign(axis.z) * extents.z);
		}

		static void computeMass(const Collider &collider, Mass::Computer &computer)
		{
			computer.setBox(collider.shape.extents, collider.pose.position, collider.pose.rotation, collider.density);
		}

		static bool hasRotation() { return true; }

	private:
		BoxCollider() = delete;
	};
}
#endif // BOX_COLLIDER_H