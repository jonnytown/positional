#ifndef MOTOR_CONSTRAINT_H
#define MOTOR_CONSTRAINT_H

#include "Constraint.h"

using namespace std;
namespace Positional
{

	struct MotorConstraint final
	{
		struct Data final
		{
			Quat rotation;
			Float torque;

			Data() = default;
			void init(
				const Quat &_rotation,
				const Float &_torque)
			{
				rotation = _rotation;
				torque = _torque;
			}
		};

		static void applyForces(Constraint &constraint, const Float &dt);
		static void solvePositions(Constraint &constraint, const Float &dtInvSq);
		static void solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq);

	private:
		MotorConstraint() = delete;
	};
}
#endif // MOTOR_CONSTRAINT_H