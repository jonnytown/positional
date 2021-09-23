/*
 * Collision pair result datum
 */
#ifndef COLLISION_RESULT_H
#define COLLISIONT_RESULT_H

#include "collision/collider/Collider.h"
#include "ContactPoint.h"
#include <vector>

namespace Positional
{
	struct CollisionResult
	{
		Store<Collider>::Ref first;
		Store<Collider>::Ref second;
		ContactPoint contact;

		CollisionResult(const Store<Collider>::Ref &_first, const Store<Collider>::Ref &_second, const ContactPoint &_contact)
			: first(_first),
			  second(_second),
			  contact(_contact) {}
	};
}
#endif // COLLISION_RESULT_H
