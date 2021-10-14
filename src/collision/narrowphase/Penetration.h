/*
 * A static class of penetration algorithms
 * A very special thanks to Ming-Lun Chou for their usefuly physics series: http://allenchou.net/game-physics-series/
 */
#ifndef PENETRATION_H
#define PENETRATION_H

#include "math/Math.h"
#include "Simplex.h"
#include "Polytope.h"
#include "ContactPoint.h"
#include <functional>

namespace Positional
{
	struct Collider;
	struct BoxCollider;
	struct SphereCollider;
	struct CapsuleCollider;

	namespace Collision
	{
		static struct Penetration 
		{
			static bool sphereSphere(const Collider &a, const Collider &b, ContactPoint &outContact);
			static bool capsuleCapsule(const Collider &a, const Collider &b, ContactPoint &outContact);
			static bool boxSphere(const Collider &box, const Collider &sphere, const bool &swapped, ContactPoint &outContact);
			static bool sphereCapsule(const Collider &sphere, const Collider &capsule, const bool &swapped, ContactPoint &outContact);

			static bool gjk_epa(const Collider &a, const Collider &b, ContactPoint &outContact);
			static bool gjk(const Collider& a, const Collider& b, Simplex& outSimplex);
			static void epa(const Collider &a, const Collider &b, const UInt8 &count, Polytope &outPolytope, ContactPoint &outContact);

		private:
			Penetration() = delete;
		};
	}
}
#endif // PENETRATION_H