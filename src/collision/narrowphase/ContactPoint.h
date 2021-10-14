/*
 * Contact point datum
 */
#ifndef CONTACT_POINT_H
#define CONTACT_POINT_H

#include "collision/collider/Collider.h"

namespace Positional
{
	struct ContactPoint
	{
		// in local body space
		Vec3 pointA;
		Vec3 pointB;
		// in world space
		Vec3 normal;
		Float depth;
		Float force;
	};
}
#endif // CONTACT_POINT_H
