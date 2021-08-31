/*
 * Contact point datum
 */
#ifndef CONTACT_POINT_H
#define CONTACT_POINT_H

#include "collision/collider/ACollider.h"

namespace Positional::Collision
{
	class ContactPoint
	{
	public:
		Vec3 point;
		Vec3 normal;
		Vec3 penetration;
		Vec3 localA;
		Vec3 localB;
	};
}
#endif // CONTACT_POINT_H
