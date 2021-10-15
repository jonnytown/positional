#ifndef JOINT_CONSTRAINT_H
#define JOINT_CONSTRAINT_H

#include "Constraint.h"

using namespace std;
namespace Positional
{
	namespace DOF
	{
		const UInt8 Linear = 1;
		const UInt8 Planar = 1 << 1;
		const UInt8 Twist = 1 << 2;
		const UInt8 Swing = 1 << 3;
	}

	struct GenericJointConstraint final
	{
		struct Data final
		{
			Pose poseA;
			Pose poseB;
			UInt8 dof;
			UInt8 hasLimits;
			Float positionCompliance;
			Float positionDamping;
			Float linearLimit;
			Float rotationCompliance;
			Float rotationDamping;
			Float minTwist;
			Float maxTwist;
			Float minSwing;
			Float maxSwing;

			Data() = default;
			void init(
				const Pose &_poseA,
				const Pose &_poseB,
				const UInt8 &_dof,
				const UInt8 &_hasLimit,
				const Float &_positionCompliance,
				const Float &_positionDamping,
				const Float &_linearLimit,
				const Float &_rotationCompliance,
				const Float &_rotationDamping,
				const Float &_minTwist,
				const Float &_maxTwist,
				const Float &_minSwing,
				const Float &_maxSwing)
			{
				poseA = _poseA;
				poseB = _poseB;
				dof = _dof;
				hasLimits = _hasLimit;
				positionCompliance = _positionCompliance;
				positionDamping = _positionDamping;
				rotationDamping = _rotationDamping;
				linearLimit = _linearLimit;
				minTwist = _minTwist;
				maxTwist = _maxTwist;
				minSwing = _minSwing;
				maxSwing = _maxSwing;
			}
		};

		static void solvePositions(Constraint &constraint, const Float &dtInvSq);
		static void solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq);
	private:
		GenericJointConstraint() = delete;
	};
}
#endif // JOINT_CONSTRAINT_H