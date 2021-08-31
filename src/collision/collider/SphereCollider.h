/*
 * A sphere collider
 */
#ifndef SPHERE_COLLIDER_H
#define SPHERE_COLLIDER_H

#include "ACollider.h"
#include "mass/Volume.h"

namespace Positional
{
	class SphereCollider : public Collider
	{
	public:
		SphereCollider(const Vec3 &_center, const Float &_radius, const Float &_density)
			: Collider(_center, Quat::identity, _density)
		{
			radius = _radius;
		}

		inline virtual Bounds bounds() const
		{
			return Bounds(pointToWorld(Vec3::zero), Vec3(radius));
		}

		inline virtual bool raycast(const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance) const override
		{
			const Vec3 centerT = pointToWorld(Vec3::zero);
			return GeomUtil::raycastSphere(centerT, radius, ray.origin, ray.normal(), maxDistance, outPoint, outNormal, outDistance);
		}

		inline virtual Vec3 localSupport(const Vec3 &axis) const override
		{
			return axis * radius;
		}

		inline virtual Float volume() const override { return Mass::sphereVolume(radius); }

		inline virtual ColliderShape shape() const override { return ColliderShape::Sphere; }
	};

	
}
#endif // SPHERE_COLLIDER_H