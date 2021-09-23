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
			body.frame.velocity = body.frame.velocity + dt * gravity;
			body.frame.position = body.frame.position + dt * body.frame.velocity;
		}

		static void differentiate(Body &body, const Float &dtInv)
		{
			body.frame.velocity = (body.frame.position - body.prevFrame.position) * dtInv;
		}

		static bool hasRotation() { return false; }

	private:
		Particle() {}
	};
}
#endif // PARTICLE_H