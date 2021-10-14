#include "GJKEPANarrowphase.h"
#include "Penetration.h"

namespace Positional::Collision
{
	bool GJKEPANarrowphase::compute(const Collider &a, const Collider &b, ContactPoint &outContact) const
	{
		const UInt8 shapePair = a.shapeId() | b.shapeId();
		switch (shapePair)
		{
		case ShapeId::Sphere:
			return Penetration::sphereSphere(a, b, outContact);
		case ShapeId::Capsule:
			return Penetration::capsuleCapsule(a, b, outContact);
		case ShapeId::Box | ShapeId::Sphere:
			return a.shapeId() == ShapeId::Box
					? Penetration::boxSphere(a, b, false, outContact)
					: Penetration::boxSphere(b, a, true, outContact);
		case ShapeId::Sphere | ShapeId::Capsule:
			return a.shapeId() == ShapeId::Sphere
					? Penetration::sphereCapsule(a, b, false, outContact)
					: Penetration::sphereCapsule(b, a, true, outContact);
		default:
			return Penetration::gjk_epa(a, b, outContact);
		}
	}

	bool boxSphereNoSwap(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		return Penetration::boxSphere(a, b, false, outContact);
	}

	bool boxSphereSwap(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		return Penetration::boxSphere(b, a, true, outContact);
	}

	bool sphereCapsuleNoSwap(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		return Penetration::sphereCapsule(a, b, false, outContact);
	}

	bool sphereCapsuleSwap(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		return Penetration::sphereCapsule(b, a, true, outContact);
	}

	PenetrationFunction GJKEPANarrowphase::getComputeFunction(const Collider &a, const Collider &b) const
	{
		const UInt8 shapePair = a.shapeId() | b.shapeId();
		switch (shapePair)
		{
		case ShapeId::Sphere:
			return Penetration::sphereSphere;
		case ShapeId::Capsule:
			return Penetration::capsuleCapsule;
		case ShapeId::Box | ShapeId::Sphere:
			return a.shapeId() == ShapeId::Box
					? boxSphereNoSwap
					: boxSphereSwap;
		case ShapeId::Sphere | ShapeId::Capsule:
			return a.shapeId() == ShapeId::Sphere
					? sphereCapsuleNoSwap
					: sphereCapsuleSwap;
		default:
			return Penetration::gjk_epa;
		}
	}
}