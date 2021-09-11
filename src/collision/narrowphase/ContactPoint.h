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
		Vec3 point;
		Vec3 normal;
		Vec3 depth;
		Vec3 localA;
		Vec3 localB;

		ContactPoint(const Vec3 &_point, const Vec3 &_normal, const Float &_depth, const Vec3 &_localA, const Vec3 &_localB)
			: point(_point),
			  normal(_normal),
			  depth(_depth),
			  localA(_localA),
			  localB(_localB) {}
	};
}
#endif // CONTACT_POINT_H
