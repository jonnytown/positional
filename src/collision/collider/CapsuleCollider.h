/*
 * A capsule collider. The capsule length is always along the y-axis.
 * The rotation property can but used for other orientations.
 */
#ifndef CAPSULE_COLLIDER_H
#define CAPSULE_COLLIDER_H

#include "ACollider.h"
#include "SphereCollider.h"
#include "mass/Volume.h"

namespace Positional
{
	class CapsuleCollider : public Collider
	{
	public:
		CapsuleCollider(const Vec3 &_center, const Quat &_rotation, const Float &_radius, const Float &_length, const UInt8 &_axis, const Float &_density)
			: Collider(_center, _rotation, _density)
		{
			radius = _radius;
			length = _length;
		}

		inline virtual Bounds bounds() const
		{
			const Float l_2 = length * 0.5;
			const Vec3 c0 = pointToWorld(Vec3(0, -l_2, 0));
			const Vec3 c1 = pointToWorld(Vec3(0, l_2, 0));

			Bounds bounds = Bounds(c0, Vec3(radius));
			bounds.merge(Bounds(c1, Vec3(radius)));
			return bounds;
		}

		inline virtual bool raycast(const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance) const override
		{
			const Float l_2 = length * 0.5;
			const Vec3 c0 = pointToWorld(Vec3(0, -l_2, 0));
			const Vec3 c1 = pointToWorld(Vec3(0, l_2, 0));
			return GeomUtil::raycastCapsule(c0, c1, length, radius, ray.origin, ray.normal(), maxDistance, outPoint, outNormal, outDistance);
		}

		inline virtual Vec3 localSupport(const Vec3 &axis) const override
		{
			const Float l_2 = length * 0.5;
			const Vec3 c[2] = {Vec3(0, -l_2, 0), Vec3(0, l_2, 0)};

			const Float dot0 = axis.dot(c[0]);
			const Float dot1 = axis.dot(c[1]);
			
			if (Math::approx(dot0, dot1))
			{
				return axis * radius;
			}

			return c[dot1 > dot0] + axis * radius;
		}

		inline virtual Float volume() const override { Mass::capsuleVolume(radius, length); }

		inline virtual ColliderShape shape() const override { return ColliderShape::Capsule; }
	};
}
#endif // CAPSULE_COLLIDER_H