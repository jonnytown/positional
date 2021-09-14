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
		Vec3 pointA;
		Vec3 pointB;
		Vec3 localA;
		Vec3 localB;
		Vec3 normal;
		Float depth;
	};
}
#endif // CONTACT_POINT_H
