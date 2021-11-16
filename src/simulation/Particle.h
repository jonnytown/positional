#ifndef PARTICLE_H
#define PARTICLE_H

#include "Body.h"
#include "collision/collider/SphereCollider.h"

namespace Positional
{
	struct Particle final
	{
		static void integrate(Body &body, const Float &dt, const Vec3 &gravity)
		{
			// TODO: apply all external forces
			body.velocity.linear = body.velocity.linear + dt * gravity + dt * body.invMass * body.forces.linear;
			body.pose.position = body.pose.position + dt * body.velocity.linear;
		}

		static void differentiate(Body &body, const Float &dtInv)
		{
			body.velocity.linear = (body.pose.position - body.prePose.position) * dtInv;
		}

		static bool hasRotation() { return false; }

	private:
		Particle() {}
	};
}
#endif // PARTICLE_H