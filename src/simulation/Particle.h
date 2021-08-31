#ifndef PARTICLE_H
#define PARTICLE_H

#include "ABody.h"
#include "collision/collider/SphereCollider.h"

namespace Positional
{
	class Particle : public ABody
	{
	private:
		SphereCollider m_collider;
	public:
		Particle(const Vec3 &_position, const Float &radius, const Float &density) : m_collider(Vec3::zero, radius, density)
		{
			position = _position;
			internal_addCollider(&m_collider);
		}

		virtual Vec3 pointToWorld(const Vec3 &point) const override { return position + point; }
		virtual Vec3 pointToLocal(const Vec3 &point) const override { return point - position; }
		virtual Vec3 vectorToWorld(const Vec3 &vector) const override { return vector; }
		virtual Vec3 vectorToLocal(const Vec3 &vector) const override { return vector; }
	};
}
#endif // PARTICLE_H