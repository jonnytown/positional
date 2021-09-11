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
		Store<Collider>::Ptr first;
		Store<Collider>::Ptr second;
		std::vector<ContactPoint> contacts;

		CollisionResult(const Store<Collider>::Ptr &_first, const Store<Collider>::Ptr &_second)
			: first(_first),
			  second(_second) {}
	};
}
#endif // COLLISION_RESULT_H
