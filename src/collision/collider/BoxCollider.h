/*
 * A box collider
 */
#ifndef BOX_COLLIDER_H
#define BOX_COLLIDER_H

#include "ACollider.h"
#include "mass/Volume.h"

namespace Positional
{
	class BoxCollider : public Collider
	{
	public:
		BoxCollider(const Vec3 &_center, const Vec3 &_extents, const Float &_density)
			: Collider(_center, Quat::identity, _density)
		{
			extents = _extents;
		}
	
		inline virtual Bounds bounds() const {
			Bounds bounds(pointToWorld(extents), Vec3::zero);
			bounds.merge(pointToWorld(Vec3(extents.x, -extents.y, extents.z)));
			bounds.merge(pointToWorld(Vec3(extents.x, -extents.y, -extents.z)));
			bounds.merge(pointToWorld(Vec3(extents.x, extents.y, -extents.z)));

			bounds.merge(pointToWorld(Vec3(-extents.x, extents.y, extents.z)));
			bounds.merge(pointToWorld(Vec3(-extents.x, -extents.y, extents.z)));
			bounds.merge(pointToWorld(-extents));
			bounds.merge(pointToWorld(Vec3(-extents.x, extents.y, -extents.z)));
			return bounds;
		}

		inline virtual bool raycast(const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance) const override
		{
			Ray rayT(pointToLocal(ray.origin), vectorToLocal(ray.normal()));
			if (GeomUtil::raycastBox(extents, rayT.origin, rayT.normal(), rayT.invNormal(), maxDistance, outPoint, outNormal, outDistance))
			{
				outPoint = pointToWorld(outPoint);
				outNormal = vectorToWorld(outNormal);
				return true;
			}
			return false;
		}

		inline virtual Vec3 localSupport(const Vec3 &axis) const override
		{
			return Vec3(
				Math::sign(axis.x) * extents.x,
				Math::sign(axis.y) * extents.y,
				Math::sign(axis.z) * extents.z);
		}

		inline virtual Float volume() const override { return Mass::boxVolume(extents); }

		inline virtual ColliderShape shape() const override { return ColliderShape::Box; }
	};
}
#endif // BOX_COLLIDER_H