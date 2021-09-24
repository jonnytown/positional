#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Body.h"

namespace Positional
{
	struct RigidBody final
	{
		static void integrate(Body &body, const Float &dt, const Vec3 &gravity)
		{
			// TODO: apply all external forces
			body.frame.velocity = body.frame.velocity + dt * gravity;
			body.frame.position = body.frame.position + dt * body.frame.velocity;

			// TODO: apply external torques
			body.applyRotation(body.frame.angularVelocity, dt);
		}

		static void differentiate(Body &body, const Float &dtInv)
		{
			body.frame.velocity = (body.frame.position - body.prevFrame.position) * dtInv;
			const Quat dq = body.frame.rotation * body.prevFrame.rotation.inverse();
			const Float dtInv2 = 2 * dtInv;
			body.frame.angularVelocity = dq.w >= 0 ?
				Vec3(dq.x * dtInv2, dq.y * dtInv2, dq.z * dtInv2) :
				Vec3(-dq.x * dtInv2, -dq.y * dtInv2, -dq.z * dtInv2);
		}

		static bool hasRotation() { return true; }

	private:
		RigidBody() {}
	};
}
#endif // RIGIDBODY_H