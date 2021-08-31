/*
 * A static class of penetration algorithms
 */
#ifndef PENETRATION_H
#define PENETRATION_H

#include "math/Math.h"

namespace Positional
{
	class Collider;
	class BoxCollider;
	class SphereCollider;
	class CapsuleCollider;

	static class Penetration 
	{
	private:
		Penetration() = delete;
	public:
		static bool compute(const Collider *const a, const Collider *const b, Float &outDepth, Vec3 &outNorm);

		static bool sphereSphere(const SphereCollider *const a, const SphereCollider *const b, Float &outDepth, Vec3 &outNorm);
		static bool capsuleCapsule(const CapsuleCollider *const a, const CapsuleCollider *const b, Float &outDepth, Vec3 &outNorm);
		static bool boxSphere(const BoxCollider *const box, const SphereCollider *const sphere, const bool &swapped, Float &outDepth, Vec3 &outNorm);
		static bool sphereCapsule(const SphereCollider *const sphere, const CapsuleCollider *const capsule, const bool &swapped, Float &outDepth, Vec3 &outNorm);

		static bool GJK_EPA(const Collider *const a, const Collider *const b, Float &outDepth, Vec3 &outNormal);
	};
}
#endif // PENETRATION_H