#include "MotorConstraint.h"

namespace Positional
{
	void MotorConstraint::applyForces(Constraint &constraint, const Float &dt)
	{
		Data *data = constraint.getData<Data>();

		if (Math::abs(data->torque) > Math::Epsilon)
		{
			Quat rot = data->rotation;
			if (constraint.bodyA.valid())
			{
				rot = constraint.bodyA.get().pose.rotation * rot;
			}

			const Vec3 torque = rot * Vec3(data->torque, 0, 0);

			if (constraint.bodyA.valid())
			{
				constraint.bodyA.get().externalForces.angular = constraint.bodyA.get().externalForces.angular + torque;
			}

			if (constraint.bodyB.valid())
			{
				constraint.bodyB.get().externalForces.angular = constraint.bodyB.get().externalForces.angular - torque;
			}
		}
	}

	void MotorConstraint::solvePositions(Constraint &constraint, const Float &dtInvSq)
	{

	}

	void MotorConstraint::solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq)
	{
		
	}
} // namespace Positional