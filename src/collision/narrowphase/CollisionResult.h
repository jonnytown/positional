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
		std::vector<ContactPoint> contacts;

		CollisionResult(const Store<Collider>::Ref &_first, const Store<Collider>::Ref &_second)
			: first(_first),
			  second(_second) {}
	};
}
#endif // COLLISION_RESULT_H
