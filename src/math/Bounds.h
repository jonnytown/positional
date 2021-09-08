/* 
 * An axis-aligned bounding box class
 */
#include "Primitives.h"
#include "Vec3.h"
#include "Ray.h"
#include "Util.h"

#ifndef BOUNDS_H
#define BOUNDS_H

namespace Positional
{
	class Bounds
	{
	public:
		Vec3 center;
	private:
		Vec3 m_extents;
	public:
		Bounds()
			: center(Vec3::zero),
			  m_extents(Vec3::zero) {}

		Bounds(const Vec3 &_center, const Vec3 &_extents)
			: center(_center),
			  m_extents(Vec3(Math::abs(_extents.x), Math::abs(_extents.y), Math::abs(_extents.z))) {}

		Bounds &operator=(const Bounds &rhs)
		{
			m_extents = rhs.m_extents;
			center = rhs.center;
			return *this;
		}

		inline const Vec3 &extents() const
		{
			return m_extents;
		}

		inline void setExtents(const Vec3 &extents)
		{
			m_extents.x = Math::abs(extents.x);
			m_extents.y = Math::abs(extents.y);
			m_extents.z = Math::abs(extents.z);
		}

		inline Vec3 min() const
		{
			return Vec3(center.x - m_extents.x, center.y - m_extents.y, center.z - m_extents.z);
		}

		inline Vec3 max() const
		{
			return Vec3(center.x + m_extents.x, center.y + m_extents.y, center.z + m_extents.z);
		}

		inline Float surfaceArea() const
		{
			return 8.0 * (m_extents.x * m_extents.y + m_extents.x * m_extents.z + m_extents.y * m_extents.z);
		}

		inline Float volume() const
		{
			return 8.0 * m_extents.x * m_extents.y * m_extents.z ;
		}

		inline bool contains(const Vec3 &point) const
		{
			return  Math::abs(point.x - center.x) <= m_extents.x
				&&  Math::abs(point.y - center.y) <= m_extents.y
				&&  Math::abs(point.z - center.z) <= m_extents.z;
		}

		inline bool containsExclusive(const Vec3 &point) const
		{
			return Math::abs(point.x - center.x) < m_extents.x
				&& Math::abs(point.y - center.y) < m_extents.y
				&& Math::abs(point.z - center.z) < m_extents.z;
		}

		inline bool contains(const Vec3 &point, const bool &exclusive) const
		{
			return exclusive ? containsExclusive(point) : contains(point);
		}

		inline bool contains(const Bounds &other) const
		{
			const Vec3 delta = (other.center - center).abs() + other.m_extents;
			return (delta.x <= m_extents.x && delta.y <= m_extents.y && delta.z <= m_extents.z);
		}

		inline bool containsExclusive(const Bounds &other) const
		{
			const Vec3 delta = (other.center - center).abs() + other.m_extents;
			return (delta.x < m_extents.x && delta.y < m_extents.y && delta.z < m_extents.z);
		}

		inline bool contains(const Bounds &other, const bool &exclusive) const
		{
			return exclusive ? containsExclusive(other) : contains(other);
		}

		inline Vec3 nearest(const Vec3 &point) const
		{
			return Vec3(
				Math::clamp(point.x, center.x - m_extents.x, center.x + m_extents.x),
				Math::clamp(point.y, center.y - m_extents.y, center.y + m_extents.y),
				Math::clamp(point.z, center.z - m_extents.z, center.z + m_extents.z));
		}

		bool intersects(const Bounds &other) const
		{
			const Vec3 delta = other.center - center;
			const Vec3 sumExtents = other.m_extents + m_extents;

			return (Math::abs(delta.x) <= sumExtents.x && Math::abs(delta.y) <= sumExtents.y && Math::abs(delta.z) <= sumExtents.z);
		}

		bool intersectsExclusive(const Bounds &other) const
		{
			const Vec3 delta = other.center - center;
			const Vec3 sumExtents = other.m_extents + m_extents;

			return (Math::abs(delta.x) < sumExtents.x && Math::abs(delta.y) < sumExtents.y && Math::abs(delta.z) < sumExtents.z);
		}

		inline bool intersects(const Bounds &other, const bool &exclusive) const
		{
			return exclusive ? intersectsExclusive(other) : intersects(other);
		}

		/*
		 * Negative distance means ray starts inside bounds
		 */
		bool intersects(const Ray &ray, Float &outDistance) const
		{
			Vec3 invDir = ray.invNormal();

			Float t1 = ((center.x - m_extents.x) - ray.origin.x) * invDir.x;
			Float t2 = ((center.x + m_extents.x) - ray.origin.x) * invDir.x;
			Float t3 = ((center.y - m_extents.y) - ray.origin.y) * invDir.y;
			Float t4 = ((center.y + m_extents.y) - ray.origin.y) * invDir.y;
			Float t5 = ((center.z - m_extents.z) - ray.origin.z) * invDir.z;
			Float t6 = ((center.z + m_extents.z) - ray.origin.z) * invDir.z;

			Float tmin = Math::max(Math::max(Math::min(t1, t2), Math::min(t3, t4)), Math::min(t5, t6));
			Float tmax = Math::min(Math::min(Math::max(t1, t2), Math::max(t3, t4)), Math::max(t5, t6));
			
			outDistance = tmin;
			return tmax >= tmin;
		}

		Bounds &merge(const Vec3 &point)
		{
			Float minX = Math::min(center.x - m_extents.x, point.x);
			Float minY = Math::min(center.y - m_extents.y, point.y);
			Float minZ = Math::min(center.z - m_extents.z, point.z);

			Float maxX = Math::max(center.x + m_extents.x, point.x);
			Float maxY = Math::max(center.y + m_extents.y, point.y);
			Float maxZ = Math::max(center.z + m_extents.z, point.z);

			m_extents = Vec3((maxX - minX) * 0.5, (maxY - minY) * 0.5, (maxZ - minZ) * 0.5);
			center = Vec3(minX + m_extents.x, minY + m_extents.y, minZ + m_extents.z);

			return *this;
		}

		Bounds &merge(const Bounds &other)
		{
			Float minX = Math::min(center.x - m_extents.x, other.center.x - other.m_extents.x);
			Float minY = Math::min(center.y - m_extents.y, other.center.y - other.m_extents.y);
			Float minZ = Math::min(center.z - m_extents.z, other.center.z - other.m_extents.z);

			Float maxX = Math::max(center.x + m_extents.x, other.center.x + other.m_extents.x);
			Float maxY = Math::max(center.y + m_extents.y, other.center.y + other.m_extents.y);
			Float maxZ = Math::max(center.z + m_extents.z, other.center.z + other.m_extents.z);

			m_extents = Vec3((maxX - minX) * 0.5, (maxY - minY) * 0.5, (maxZ - minZ) * 0.5);
			center = Vec3(minX + m_extents.x, minY + m_extents.y, minZ + m_extents.z);

			return *this;
		}

		/*
		 * Expand each side by amound
		 */
		inline Bounds &expand(const Float &amount)
		{
			m_extents = Vec3(Math::abs(m_extents.x + amount), Math::abs(m_extents.y + amount), Math::abs(m_extents.z + amount));
			return *this;
		}

		/*
		 * Expand each side by the corresponding amount component.
		 */
		inline Bounds &expand(const Vec3 &amount)
		{
			m_extents = Vec3(Math::abs(m_extents.x + amount.x), Math::abs(m_extents.y + amount.y), Math::abs(m_extents.z + amount.z));
			return *this;
		}

		Bounds merged(const Vec3 &point) const
		{
			Float minX = Math::min(center.x - m_extents.x, point.x);
			Float minY = Math::min(center.y - m_extents.y, point.y);
			Float minZ = Math::min(center.z - m_extents.z, point.z);

			Float maxX = Math::max(center.x + m_extents.x, point.x);
			Float maxY = Math::max(center.y + m_extents.y, point.y);
			Float maxZ = Math::max(center.z + m_extents.z, point.z);

			Vec3 _extents((maxX - minX) * 0.5, (maxY - minY) * 0.5, (maxZ - minZ) * 0.5);
			Vec3 _center(minX + _extents.x, minY + _extents.y, minZ + _extents.z);

			return Bounds(_center, _extents);
		}

		Bounds merged(const Bounds &other) const
		{
			Float minX = Math::min(center.x - m_extents.x, other.center.x - other.m_extents.x);
			Float minY = Math::min(center.y - m_extents.y, other.center.y - other.m_extents.y);
			Float minZ = Math::min(center.z - m_extents.z, other.center.z - other.m_extents.z);

			Float maxX = Math::max(center.x + m_extents.x, other.center.x + other.m_extents.x);
			Float maxY = Math::max(center.y + m_extents.y, other.center.y + other.m_extents.y);
			Float maxZ = Math::max(center.z + m_extents.z, other.center.z + other.m_extents.z);

			Vec3 _extents((maxX - minX) * 0.5, (maxY - minY) * 0.5, (maxZ - minZ) * 0.5);
			Vec3 _center(minX + _extents.x, minY + _extents.y, minZ + _extents.z);

			return Bounds(_center, _extents);
		}

		inline Bounds expanded(const Float &amount) const
		{
			return Bounds(center, Vec3(m_extents.x + amount, m_extents.y + amount, m_extents.z + amount));
		}

		/*
		 * Expand each side by the corresponding amount component.
		 */
		inline Bounds expanded(const Vec3 &amount) const
		{
			return Bounds(center, Vec3(m_extents.x + amount.x, m_extents.y + amount.y, m_extents.z + amount.z));
		}
	};
}

#endif // BOUNDS_H