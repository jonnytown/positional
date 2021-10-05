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
		Ref<Collider> first;
		Ref<Collider> second;
		ContactPoint contact;

		CollisionResult(const Ref<Collider> &_first, const Ref<Collider> &_second, const ContactPoint &_contact)
			: first(_first),
			  second(_second),
			  contact(_contact) {}
	};
}
#endif // COLLISION_RESULT_H
