#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include "math/Math.h"
#include "Constraint.h"
#include "collision/narrowphase/INarrowphase.h"
#include <functional>

using namespace std;
namespace Positional
{
	struct ContactConstraint final
	{
		struct Data final
		{
		private:
			Collision::PenetrationFunction m_compute;

		public:
			Ref<Collider> colliderA;
			Ref<Collider> colliderB;
			
			bool colliding;
			Float staticFriction;
			Float dynamicFriction;
			Float restitution;
			ContactPoint contact;

			Data() = default;
			inline void init(const Ref<Collider> &_colliderA, const Ref<Collider> &_colliderB, const Collision::INarrowphase *narrowphase)
			{
				const Collider &collA = _colliderA.get();
				const Collider &collB = _colliderB.get();

				m_compute = narrowphase->getComputeFunction(collA, collB);

				colliderA = _colliderA;
				colliderB = _colliderB;
				colliding = false;
				staticFriction = (collA.staticFriction + collB.staticFriction) * 0.5;
				dynamicFriction = (collA.dynamicFriction + collB.dynamicFriction) * 0.5;
				restitution = (collA.restitution + collB.restitution) * 0.5;
			}

			inline void update()
			{
				colliding = m_compute(colliderA.get(), colliderB.get(), contact);
			}
		};

		static void solvePositions(Constraint &constraint, const Float &dtInvSq);
		static void solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq);

	private:
		ContactConstraint() = delete;
	};
}
#endif // CONTACT_CONSTRAINT_H