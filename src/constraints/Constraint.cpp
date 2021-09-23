#include "Constraint.h"

namespace Positional
{
	bool Constraint::computeCorrections(
		const Vec3 &correction,
		const Float &compliance,
		const Float &dtInvSq,
		Vec3 &outNormal,
		Float &outLambda,
		const optional<Vec3> &posA,
		const optional<Vec3> &posB) const
	{
		const Float cSq = correction.lengthSq();
		if (cSq <= 0)
		{
			return false;
		}

		const Float C = Math::sqrt(cSq);
		outNormal = correction.normalized();
		
		return computeCorrections(outNormal, C, compliance, dtInvSq, outLambda, posA, posB);
	}

	bool Constraint::computeCorrections(
		const Vec3 &correctionNormal,
		const Float& correctionLength,
		const Float& compliance,
		const Float& dtInvSq,
		Float& outLambda,
		const optional<Vec3>& posA,
		const optional<Vec3>& posB) const
	{
		const Float w0 = bodyA.valid() ? bodyA.get().getInverseMass(correctionNormal, posA) : 0.0;
		const Float w1 = bodyB.valid() ? bodyB.get().getInverseMass(correctionNormal, posB) : 0.0;
		const Float w = w0 + w1;

		if (w == 0.0)
		{
			return false;
		}

		outLambda = -correctionLength / (w + compliance * dtInvSq);
		return true;
	}
}