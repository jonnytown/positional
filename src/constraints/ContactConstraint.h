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
			Store<Collider>::Ref colliderA;
			Store<Collider>::Ref colliderB;
			bool colliding;
			Float staticFriction;
			Float dynamicFriction;
			Float restitution;
			ContactPoint contact;
			Float force;
			Vec3 prevVelocity;

			Data() = default;
			void init(const Store<Collider>::Ref &_colliderA, const Store<Collider>::Ref &_colliderB);
		};

		static void solvePositions(Constraint &constraint, const Float &dtInvSq);
		static void solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq);

	private:
		ContactConstraint() = delete;
	};
}
#endif // CONTACT_CONSTRAINT_H