/*
 * Collision pair result datum
 */
#ifndef COLLISION_RESULT_H
#define COLLISIONT_RESULT_H

#include "collision/collider/ACollider.h"
#include "ContactPoint.h"
#include <vector>

namespace Positional::Collision
{
	class CollisionResult
	{
	public:
		Collider *first;
		Collider *second;
		std::vector<ContactPoint> contacts;
	};
}
#endif // COLLISION_RESULT_H
