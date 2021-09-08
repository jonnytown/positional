/*
 * A static class of penetration algorithms
 */
#ifndef PENETRATION_H
#define PENETRATION_H

#include "math/Math.h"

namespace Positional
{
	struct Collider;
	struct BoxCollider;
	struct SphereCollider;
	struct CapsuleCollider;

	static struct Penetration 
	{
		static bool compute(const Collider &a, const Collider &b, Float &outDepth, Vec3 &outNorm);

		static bool sphereSphere(const Collider &a, const Collider &b, Float &outDepth, Vec3 &outNorm);
		static bool capsuleCapsule(const Collider &a, const Collider &b, Float &outDepth, Vec3 &outNorm);
		static bool boxSphere(const Collider &box, const Collider &sphere, const bool &swapped, Float &outDepth, Vec3 &outNorm);
		static bool sphereCapsule(const Collider &sphere, const Collider &capsule, const bool &swapped, Float &outDepth, Vec3 &outNorm);

		static bool GJK_EPA(const Collider &a, const Collider &b, Float &outDepth, Vec3 &outNormal);

	private:
		Penetration() = delete;
	};
}
#endif // PENETRATION_H