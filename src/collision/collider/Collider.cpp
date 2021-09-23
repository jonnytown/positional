#include "Collider.h"
#include "collision/narrowphase/Penetration.h"
#include "simulation/Body.h"

namespace Positional
{
	Vec3 Collider::pointToWorld(const Vec3 &point) const
	{
		Vec3 bodySpace = pose.transform(point);
		if (m_body.valid())
		{
			return m_body.get().frame.transform(bodySpace);
		}
		return bodySpace;
	}

	Vec3 Collider::vectorToWorld(const Vec3 &vector) const
	{
		Vec3 bodySpace = pose.rotate(vector);
		if (m_body.valid())
		{
			return m_body.get().frame.rotate(bodySpace);
		}
		return bodySpace;
	}

	Vec3 Collider::pointToLocal(const Vec3 &point) const
	{
		Vec3 bodySpace = m_body.valid() ? m_body.get().frame.inverseTransform(point) : point;
		return pose.inverseTransform(bodySpace);
	}

	Vec3 Collider::vectorToLocal(const Vec3 &vector) const
	{
		Vec3 bodySpace = m_body.valid() ? m_body.get().frame.inverseRotate(vector) : vector;
		return pose.inverseRotate(bodySpace);
	}
}