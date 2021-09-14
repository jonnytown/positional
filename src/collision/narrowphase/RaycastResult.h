/*
 * Raycast result datum
 */
#ifndef RAYCAST_RESULT_H
#define RAYCAST_RESULT_H

#include "collision/collider/Collider.h"

namespace Positional
{
	struct RaycastResult
	{
		Store<Collider>::Ref collider;
		Vec3 point;
		Vec3 normal;
		Float distance;

		RaycastResult(const Store<Collider>::Ref &_collider, const Vec3 &_point, const Vec3 &_normal, const Float &_distance)
			: collider(_collider),
			  point(_point),
			  normal(_normal),
			  distance(_distance) {}
	};
}
#endif // RAYCAST_RESULT_H
