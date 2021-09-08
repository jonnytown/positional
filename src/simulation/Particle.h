#ifndef PARTICLE_H
#define PARTICLE_H

#include "Body.h"
#include "collision/collider/SphereCollider.h"

namespace Positional
{
	struct Particle final
	{
		static Vec3 pointToWorld(const Body &body, const Vec3 &point) { return body.position + point; }
		static Vec3 pointToLocal(const Body &body, const Vec3 &point) { return point - body.position; }
		static Vec3 vectorToWorld(const Body &body, const Vec3 &vector) { return vector; }
		static Vec3 vectorToLocal(const Body &body, const Vec3 &vector) { return vector; }

	private:
		Particle() {}
	};
}
#endif // PARTICLE_H