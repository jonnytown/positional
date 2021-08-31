/*
 * Raycast result datum
 */
#ifndef RAYCAST_RESULT_H
#define RAYCAST_RESULT_H

#include "collision/collider/ACollider.h"

namespace Positional::Collision
{
	class RaycastResult
	{
	public:
		Collider *collider;
		Vec3 point;
		Vec3 normal;
		Float distance;
	};
}
#endif // RAYCAST_RESULT_H
