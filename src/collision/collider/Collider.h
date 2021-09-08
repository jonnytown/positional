#ifndef COLLIDER_H
#define COLLIDER_H

#include "math/Math.h"
#include "data/Store.h"
#include <optional>
#include <functional>
#include "ShapeId.h"

using namespace std;

namespace Positional
{
	struct Body;

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
		Store<Body>::Ptr m_body;
		bool m_isStatic;
		UInt8(*m_shapeId)();
		Bounds(*m_bounds)(const Collider &);
		Float(*m_volume)(const Collider &);
		bool(*m_raycast)(const Collider &, const Ray &, const Float &, Vec3 &, Vec3 &, Float &);
		Vec3(*m_support)(const Collider &, const Vec3 &);

	public:
		Vec3 center;
		Quat rotation;
		UInt32 mask;
		Float density;
		Shape shape;

		Collider(Store<Body>::Ptr body,const Vec3 &_center, const Quat &_rotation, const Shape &_shape)
			: m_body(body),
			  m_isStatic(!body.valid()),
			  m_shapeId(NULL),
			  m_bounds(NULL),
			  m_volume(NULL),
			  m_raycast(NULL),
			  m_support(NULL),
			  center(_center),
			  rotation(_rotation),
			  mask(0xffffffffui32),
			  density(1),
			  shape(_shape) {}

		Store<Body>::Ptr body() const { return m_body; };

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
		 * shape id of the collider
		 */
		UInt8 shapeId() const { return m_shapeId(); }

		Vec3 pointToWorld(const Vec3& point) const;
		Vec3 pointToLocal(const Vec3& point) const;

		Vec3 vectorToWorld(const Vec3& vector) const;
		Vec3 vectorToLocal(const Vec3& vector) const;

		template <typename T>
		inline static Collider create(Store<Body>::Ptr body, const Vec3 &center, const Quat &rotation, const Shape &shape)
		{
			Collider collider = Collider(body, center, rotation, shape);
			collider.m_shapeId = T::shapeId;
			collider.m_bounds = T::bounds;
			collider.m_volume = T::volume;
			collider.m_raycast = T::raycast;
			collider.m_support = T::localSupport;
			return collider;
		}
	};
}
#endif // COLLIDER_H