#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Body.h"

namespace Positional
{
	struct RigidBody final
	{
		static void integrate(Body &body, const Float &dt, const Vec3 &gravity)
		{
			body.velocity.linear += dt * gravity + dt * body.invMass * body.forces.linear;
			body.pose.position += dt * body.velocity.linear;

			Vec3 dOmega = body.massPose.inverseRotate(body.pose.inverseRotate(body.forces.angular * dt));
			dOmega.x *= body.invInertia.x;
			dOmega.y *= body.invInertia.y;
			dOmega.z *= body.invInertia.z;
			dOmega = body.pose.rotate(body.massPose.rotate(dOmega));

			body.velocity.angular += dOmega;
			body.applyRotation(body.velocity.angular, dt);
		}

		static void differentiate(Body &body, const Float &dtInv)
		{
			body.velocity.linear = (body.pose.transform(body.massPose.position) - body.prePose.transform(body.massPose.position)) * dtInv;
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