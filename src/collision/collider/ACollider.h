#ifndef ACOLLIDER_H
#define ACOLLIDER_H

#include "math/Math.h"
#include <optional>

namespace Positional
{
	enum ColliderShape : UInt8
	{
		Box = 1 << 0,
		Sphere = 1 << 1,
		Capsule = 1 << 2,
		Hull = 1 << 3
	};

	class Collider
	{
	friend class ABody;
	protected:
		std::optional<ABody *>m_body;
	public:
		Vec3 center;
		Quat rotation;
		UInt32 mask;
		Float density;

		// shape union
		union
		{
			Vec3 extents;
			struct
			{
				Float radius;
				Float length;
			};
			void *mesh; // placeholder for mesh pointer
		};

		Collider(const Vec3 &_center, const Quat &_rotation, const Float &_density)
		: center(_center), rotation(_rotation), mask(0xffffffffui32), density(_density) {}

		const std::optional<ABody *> &body() const { return m_body; };

		/*
		 * Bounding box in world space
		 */
		virtual Bounds bounds() const = 0;

		/*
		 * Does a ray hit the collider
		 */
		virtual bool raycast(const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance) const = 0;

		/*
		 * Support function for GJK and EPA algorithms. Returns farthest point on surface along normal axis.
		 */
		virtual Vec3 localSupport(const Vec3 &axis) const = 0;

		/*
		 * Volume of the collider
		 */
		virtual Float volume() const = 0;

		/*
		 * shape id of the collider
		 */
		virtual ColliderShape shape() const = 0;

		Vec3 pointToWorld(const Vec3& point) const;
		Vec3 pointToLocal(const Vec3& point) const;

		Vec3 vectorToWorld(const Vec3& vector) const;
		Vec3 vectorToLocal(const Vec3& vector) const;
	};
}
#endif // ACOLLIDER_H