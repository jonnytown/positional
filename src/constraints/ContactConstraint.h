#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include "math/Math.h"
#include "Constraint.h"
#include "collision/narrowphase/ContactPoint.h"

using namespace std;
namespace Positional
{
	struct ContactConstraint final
	{
		struct Data
		{
			Ref<Collider> colliderA;
			Ref<Collider> colliderB;
			bool colliding;
			Float staticFriction;
			Float dynamicFriction;
			Float restitution;
			ContactPoint contact;
			Float force;

			Data() = default;
			void init(const Ref<Collider> &_colliderA, const Ref<Collider> &_colliderB);
		};

		static void solvePositions(Constraint &constraint, const Float &dtInvSq);
		static void solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq);

	private:
		ContactConstraint() = delete;
	};
}
#endif // CONTACT_CONSTRAINT_H