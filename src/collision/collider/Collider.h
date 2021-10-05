#ifndef COLLIDER_H
#define COLLIDER_H

#include "math/Math.h"
#include "data/Store.h"
#include <optional>
#include <functional>
#include "ShapeId.h"
#include "simulation/Pose.h"

using namespace std;

namespace Positional
{
	struct Body;

	namespace Mass
	{
		struct Computer;
	}

	union Shape
	{
		Vec3 extents;
		struct
		{
			Float radius;
			Float length;
		};
		void* mesh; // placeholder for mesh pointer

		Shape() : extents(Vec3::zero)
		{
			memset(this, 0, sizeof(Shape));
		}

		Shape(const Vec3& _extents) : extents(_extents) {}

		Shape(const Float &_radius) : extents(Vec3::zero)
		{
			radius = _radius;
		}

		Shape(const Float &_radius, const Float& _length) : extents(Vec3::zero)
		{
			radius = _radius;
			length = _length;
		}

		Shape(void *_mesh) : extents(Vec3::zero)
		{
			mesh = _mesh;
		}

		Shape(const Shape &other) : extents(Vec3::zero)
		{
			memcpy(this, &other, sizeof(Shape));
		}

		Shape &operator=(const Shape &other) noexcept
		{
			memcpy(this, &other, sizeof(Shape));
			return *this;
		}
	};

	struct Collider final
	{
	private:
		Ref<Body> m_body;
		bool m_isStatic;
		UInt8 m_shapeId;
		Bounds(*m_bounds)(const Collider &);
		Float(*m_volume)(const Collider &);
		bool(*m_raycast)(const Collider &, const Ray &, const Float &, Vec3 &, Vec3 &, Float &);
		Vec3(*m_support)(const Collider &, const Vec3 &);
		void(*m_computeMass)(const Collider &, Mass::Computer &);

		Collider(
			const Ref<Body>& body,
			const UInt8& shapeId,
			const Shape& _shape,
			const Vec3& position,
			const Quat& rotation,
			const bool& hasRotation,
			const Float& _density,
			const Float& _staticFriction,
			const Float& _dynamicFriction,
			const Float& _restitution,
			Bounds(*bounds)(const Collider&),
			Float(*volume)(const Collider&),
			bool(*raycast)(const Collider&, const Ray&, const Float&, Vec3&, Vec3&, Float&),
			Vec3(*support)(const Collider&, const Vec3&),
			void(*computeMass)(const Collider&, Mass::Computer&)
		) :
			m_body(body),
			m_isStatic(!body.valid()),
			m_shapeId(shapeId),
			m_bounds(bounds),
			m_volume(volume),
			m_raycast(raycast),
			m_support(support),
			m_computeMass(computeMass),
			shape(_shape),
			pose(position, rotation, hasRotation),
			mask(0xFFFFFFFFui32),
			density(_density),
			staticFriction(_staticFriction),
			dynamicFriction(_dynamicFriction),
			restitution(_restitution)
		{
		}

	public:
		Shape shape;
		Pose pose;
		UInt32 mask;
		Float density;
		Float staticFriction;
		Float dynamicFriction;
		Float restitution;

		Ref<Body> body() const { return m_body; };

		bool isStatic() const { return m_isStatic; }

		/*
		 * Bounding box in world space
		 */
		Bounds bounds() const { return m_bounds(*this); }

		/*
		 * Does a ray hit the collider
		 */
		bool raycast(const Ray &ray, const Float &maxDistance, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance) const
		{
			return m_raycast(*this, ray, maxDistance, outPoint, outNormal, outDistance);
		}

		/*
		 * Support function for GJK and EPA algorithms. Returns farthest point on surface along normal axis.
		 */
		Vec3 localSupport(const Vec3 &axis) const
		{
			return m_support(*this, axis);
		}

		/*
		 * Volume of the collider
		 */
		Float volume() const { return m_volume(*this); }

		/*
		 * Compute mass properties
		 */
		void computeMass(Mass::Computer &computer) const { m_computeMass(*this, computer); }

		/*
		 * shape id of the collider
		 */
		UInt8 shapeId() const { return m_shapeId; }

		Vec3 pointToWorld(const Vec3 &point) const;
		Vec3 vectorToWorld(const Vec3 &vector) const;

		Vec3 pointToLocal(const Vec3 &point) const;
		Vec3 vectorToLocal(const Vec3 &vector) const;

		template <typename T>
		inline static Collider create(const Ref<Body> &body, const Vec3 &position, const Quat &rotation, const Shape &shape, const Float &density, const Float &staticFriction, const Float &dynamicFriction, const Float &bounciness)
		{			
			return Collider(
				body,
				T::shapeId(),
				shape,
				position,
				rotation,
				T::hasRotation(),
				density,
				staticFriction,
				dynamicFriction,
				bounciness,
				T::bounds,
				T::volume,
				T::raycast,
				T::localSupport,
				T::computeMass
			);
		}
	};
}
#endif // COLLIDER_H