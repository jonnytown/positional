#include "Collider.h"
#include "collision/narrowphase/Penetration.h"
#include "simulation/Body.h"

namespace Positional
{
	Vec3 Collider::pointToWorld(const Vec3& point) const
	{
		Vec3 bodySpace = rotation * point + center;
		if (m_body.valid())
		{
			return m_body.get()->pointToWorld(point);
		}
		return bodySpace;
	}

	Vec3 Collider::pointToLocal(const Vec3& point) const
	{
		Vec3 bodySpace = m_body.valid() ? m_body.get()->pointToLocal(point) : point;
		return rotation.inverse() * (bodySpace - center);
	}

	Vec3 Collider::vectorToWorld(const Vec3& vector) const
	{
		Vec3 bodySpace = rotation * vector;
		if (m_body.valid())
		{
			return m_body.get()->vectorToWorld(vector);
		}
		return bodySpace;
	}

	Vec3 Collider::vectorToLocal(const Vec3& vector) const
	{
		Vec3 bodySpace = m_body.valid() ? m_body.get()->vectorToLocal(vector) : vector;
		return rotation.inverse() * bodySpace;
	}
}