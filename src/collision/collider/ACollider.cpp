#include "ACollider.h"
#include "collision/narrowphase/Penetration.h"
#include "simulation/ABody.h"

namespace Positional
{
	Vec3 Collider::pointToWorld(const Vec3& point) const
	{
		Vec3 bodySpace = rotation * point + center;
		if (m_body.has_value())
		{
			return m_body.value()->pointToWorld(point);
		}
		return bodySpace;
	}

	Vec3 Collider::pointToLocal(const Vec3& point) const
	{
		Vec3 bodySpace = m_body.has_value() ? m_body.value()->pointToLocal(point) : point;
		return rotation.inverse() * (bodySpace - center);
	}

	Vec3 Collider::vectorToWorld(const Vec3& vector) const
	{
		Vec3 bodySpace = rotation * vector;
		if (m_body.has_value())
		{
			return m_body.value()->vectorToWorld(vector);
		}
		return bodySpace;
	}

	Vec3 Collider::vectorToLocal(const Vec3& vector) const
	{
		Vec3 bodySpace = m_body.has_value() ? m_body.value()->vectorToLocal(vector) : vector;
		return rotation.inverse() * bodySpace;
	}
}