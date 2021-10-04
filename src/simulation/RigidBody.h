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
			body.velocity.linear = body.velocity.linear + dt * gravity;
			body.pose.position = body.pose.position + dt * body.velocity.linear;

			// TODO: apply external torques
			body.applyRotation(body.velocity.angular, dt);
		}

		static void differentiate(Body &body, const Float &dtInv)
		{
			body.velocity.linear = (body.pose.position - body.prePose.position) * dtInv;
			const Quat dq = body.pose.rotation * body.prePose.rotation.inverse();
			const Float dtInv2 = 2 * dtInv;
			body.velocity.angular = dq.w >= 0 ?
				Vec3(dq.x * dtInv2, dq.y * dtInv2, dq.z * dtInv2) :
				Vec3(-dq.x * dtInv2, -dq.y * dtInv2, -dq.z * dtInv2);
		}

		static bool hasRotation() { return true; }

	private:
		RigidBody() {}
	};
}
#endif // RIGIDBODY_H