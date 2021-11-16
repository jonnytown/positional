/*
 * A capsule collider. The capsule length is always along the y-axis.
 * The rotation property can but used for other orientations.
 */
#ifndef CYLINDER_COLLIDER_H
#define CYLINDER_COLLIDER_H

#include "ShapeId.h"
#include "mass/Volume.h"
#include "mass/Computer.h"
#include "Collider.h"

namespace Positional
{
	struct CylinderCollider final
	{
		static UInt8 shapeId() { return ShapeId::Cylinder; }

		static Bounds bounds(const Collider &collider)
		{
			const Float l_2 = collider.shape.length * 0.5;
			const Vec3 axis = collider.vectorToWorld(Vec3::pos_x);

			Bounds bounds = Bounds(
				collider.pointToWorld(Vec3::zero),
				Vec3(
					Math::abs(axis.x * l_2) + collider.shape.radius * Math::sqrt(1-axis.x * axis.x),
					Math::abs(axis.y * l_2) + collider.shape.radius * Math::sqrt(1-axis.y * axis.y),
					Math::abs(axis.z * l_2) + collider.shape.radius * Math::sqrt(1-axis.z * axis.z)));
			return bounds;
		}

		static Float volume(const Collider &collider) { return Mass::cylinderVolume(collider.shape.radius, collider.shape.length); }

		static bool raycast(const Collider &collider, const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
		{
			const Float l_2 = collider.shape.length * 0.5;
			const Vec3 c0 = collider.pointToWorld(Vec3(-l_2, 0, 0));
			const Vec3 c1 = collider.pointToWorld(Vec3(l_2, 0, 0));
			const Vec3 axis = collider.vectorToWorld(Vec3::pos_x);
			return GeomUtil::raycastCylinder(c0, c1, axis, collider.shape.length, collider.shape.radius, ray.origin, ray.normal(), maxDistance, outPoint, outNormal, outDistance);
		}

		static Vec3 localSupport(const Collider &collider, const Vec3 &axis)
		{
			const Float l_2 = collider.shape.length * 0.5;
			const Vec3 c[2] = {Vec3(-l_2, 0, 0), Vec3(l_2, 0, 0)};

			const Float dot0 = axis.dot(c[0]);
			const Float dot1 = axis.dot(c[1]);

			if (Math::approx(dot0, dot1))
			{
				return axis * collider.shape.radius;
			}

			Vec3 n = axis.projectOnPlane(Vec3::pos_x);
			const Float lenSq = n.lengthSq();
			if (Math::approx(lenSq, 0))
			{
				return c[dot1 > dot0];
			}
			n.normalize();
			
			return c[dot1 > dot0] + n* collider.shape.radius;
		}

		static void computeMass(const Collider &collider, Mass::Computer &computer)
		{
			computer.setCylinder(collider.shape.radius, collider.shape.length, collider.pose.position, collider.pose.rotation, collider.density);
		}

		static bool hasRotation() { return true; }

	private:
		CylinderCollider() = delete;
	};
}
#endif // CYLINDER_COLLIDER_H