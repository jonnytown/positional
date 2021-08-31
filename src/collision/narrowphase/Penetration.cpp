#include "Penetration.h"
#include "collision/collider/ACollider.h"
#include "collision/collider/BoxCollider.h"
#include "collision/collider/SphereCollider.h"
#include "collision/collider/CapsuleCollider.h"

namespace Positional
{
	bool Penetration::compute(const Collider *const a, const Collider *const b, Float &outDepth, Vec3 &outNorm)
	{
		const UInt8 shapePair = a->shape() | b->shape();
		switch (shapePair)
		{
		case ColliderShape::Sphere:
			return sphereSphere(static_cast<const SphereCollider *const>(a), static_cast<const SphereCollider *const>(b), outDepth, outNorm);
		case ColliderShape::Capsule:
			return capsuleCapsule(static_cast<const CapsuleCollider *const>(a), static_cast<const CapsuleCollider *const>(b), outDepth, outNorm);
		case ColliderShape::Box | ColliderShape::Sphere:
			return a->shape() == ColliderShape::Box
					   ? boxSphere(static_cast<const BoxCollider *const>(a), static_cast<const SphereCollider *const>(b), false, outDepth, outNorm)
					   : boxSphere(static_cast<const BoxCollider *const>(b), static_cast<const SphereCollider *const>(a), true, outDepth, outNorm);
		case ColliderShape::Sphere | ColliderShape::Capsule:
			return a->shape() == ColliderShape::Sphere
					   ? sphereCapsule(static_cast<const SphereCollider *const>(a), static_cast<const CapsuleCollider *const>(b), false, outDepth, outNorm)
					   : sphereCapsule(static_cast<const SphereCollider *const>(b), static_cast<const CapsuleCollider *const>(a), true, outDepth, outNorm);
		default:
			return GJK_EPA(a, b, outDepth, outNorm);
		}
	}

	bool Penetration::sphereSphere(const SphereCollider *const a, const SphereCollider *const b, Float &outDepth, Vec3 &outNorm)
	{
		const Vec3 toA = a->pointToWorld(Vec3::zero) - b->pointToWorld(Vec3::zero);
		const Float rs = (a->radius + b->radius);
		const Float lenSq = toA.lengthSq();
		if (lenSq < rs * rs)
		{
			Float len = Math::sqrt(lenSq);
			outNorm = toA * (1 / len);
			outDepth = rs - len;
			return true;
		}
		return false;
	}

	bool Penetration::capsuleCapsule(const CapsuleCollider *const a, const CapsuleCollider *const b, Float &outDepth, Vec3 &outNorm)
	{
		const Float al_2 = a->length / 2;
		const Vec3 a0 = a->pointToWorld(Vec3(-al_2, 0, 0));
		const Vec3 a1 = a->pointToWorld(Vec3(al_2, 0, 0));

		const Float bl_2 = b->length / 2;
		const Vec3 b0 = b->pointToWorld(Vec3(-bl_2, 0, 0));
		const Vec3 b1 = b->pointToWorld(Vec3(bl_2, 0, 0));

		Vec3 nearA, nearB;
		GeomUtil::nearestOnSegments(a0, a1, b0, b1, nearA, nearB);
		const Vec3 toA = nearA - nearB;
		const Float rs = (a->radius + b->radius);
		const Float lenSq = toA.lengthSq();

		if (lenSq < rs * rs)
		{
			Float len = Math::sqrt(lenSq);
			outNorm = toA * (1 / len);
			outDepth = rs - len;
			return true;
		}

		return false;
	}

	bool Penetration::sphereCapsule(const SphereCollider *const sphere, const CapsuleCollider *const capsule, const bool &swapped, Float &outDepth, Vec3 &outNorm)
	{
		const Vec3 c = sphere->pointToWorld(Vec3::zero);

		const Float al_2 = capsule->length / 2;
		const Vec3 a0 = capsule->pointToWorld(Vec3(-al_2, 0, 0));
		const Vec3 a1 = capsule->pointToWorld(Vec3(al_2, 0, 0));

		const Vec3 a = GeomUtil::nearestOnSegment(c, a0, a1);
		const Vec3 toA = a - c;
		const Float rs = (sphere->radius + capsule->radius);
		const Float lenSq = toA.lengthSq();

		if (lenSq < rs * rs)
		{
			Float len = Math::sqrt(lenSq);
			outNorm = toA * (1 / len);
			if (swapped)
			{
				outNorm = -outNorm;
			}
			outDepth = rs - len;
			return true;
		}

		return false;
	}

	inline void testBoxNorm(const Float &test, const Vec3 &passNorm, Float &ioMin, Vec3 &outNorm)
	{
		if (test < ioMin)
		{
			ioMin = test;
			outNorm = passNorm;
		}
	}

	bool Penetration::boxSphere(const BoxCollider *const box, const SphereCollider *const sphere, const bool &swapped, Float &outDepth, Vec3 &outNorm)
	{
		const Vec3 centerInBoxSpace = box->pointToLocal(sphere->pointToWorld(Vec3::zero));

		const Bounds bounds(Vec3::zero, box->extents);

		if (bounds.contains(centerInBoxSpace))
		{
			const Vec3 &e = bounds.extents();
			Float min = FLOAT_MAX;
			testBoxNorm(Math::abs(e.x - centerInBoxSpace.x), Vec3::neg_x, min, outNorm);
			testBoxNorm(Math::abs(-e.x - centerInBoxSpace.x), Vec3::pos_x, min, outNorm);
			testBoxNorm(Math::abs(e.y - centerInBoxSpace.y), Vec3::neg_y, min, outNorm);
			testBoxNorm(Math::abs(-e.y - centerInBoxSpace.y), Vec3::pos_y, min, outNorm);
			testBoxNorm(Math::abs(e.z - centerInBoxSpace.y), Vec3::neg_z, min, outNorm);
			testBoxNorm(Math::abs(-e.z - centerInBoxSpace.y), Vec3::pos_z, min, outNorm);
			if (swapped)
			{
				outNorm = -outNorm;
			}
			outNorm = box->vectorToWorld(outNorm);
			outDepth = sphere->radius + min;
			return true;
		}

		const Vec3 closest = bounds.nearest(centerInBoxSpace);
		const Vec3 toClosest = closest - centerInBoxSpace;
		const Float lenSq = toClosest.lengthSq();
		if (lenSq < sphere->radius * sphere->radius)
		{
			Float len = Math::sqrt(lenSq);
			outNorm = toClosest * (1 / len);
			if (swapped)
			{
				outNorm = -outNorm;
			}
			outNorm = box->vectorToWorld(outNorm);
			outDepth = sphere->radius - len;
			return true;
		}
		return false;
	}

	bool Penetration::GJK_EPA(const Collider *const a, const Collider *const b, Float &outDepth, Vec3 &outNormal)
	{
		return false;
	}
} // namespace Positional
