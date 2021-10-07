#include "ContactConstraint.h"

#include "collision/narrowphase/Penetration.h"
#include "simulation/World.h"

namespace Positional
{
	void ContactConstraint::Data::init(const Ref<Collider> &_colliderA, const Ref<Collider> &_colliderB)
	{
		colliderA = _colliderA;
		colliderB = _colliderB;

		const Collider &collA = _colliderA.get();
		const Collider &collB = _colliderB.get();

		colliding = false;
		staticFriction = (collA.staticFriction + collB.staticFriction) * 0.5;
		dynamicFriction = (collA.dynamicFriction + collB.dynamicFriction) * 0.5;
		restitution = (collA.restitution + collB.restitution) * 0.5;

		contact = ContactPoint();
		force = 0;
	}

	inline void getContacts(const Constraint &constraint, Vec3& posA, Vec3& posB)
	{
		auto d = constraint.getData<ContactConstraint::Data>();
		posA = Body::pointToWorld(constraint.bodyA, d->contact.pointA);
		posB = Body::pointToWorld(constraint.bodyB, d->contact.pointB);
	}

	inline void getContacts(const Constraint &constraint, Vec3 &prevA, Vec3& currA, Vec3 &prevB, Vec3& currB)
	{
		auto d = constraint.getData<ContactConstraint::Data>();
		Body::pointsToWorld(constraint.bodyA, d->contact.pointA, prevA, currA);
		Body::pointsToWorld(constraint.bodyB, d->contact.pointB, prevB, currB);
	}

	inline Vec3 getVelocity(const Constraint& constraint, Vec3& posA, const Vec3& posB)
	{
		Vec3 velocity(0);
		if (constraint.bodyA.valid())
		{
			const Body &a = constraint.bodyA.get();
			velocity = a.getVelocityAt( posA);
		}

		if (constraint.bodyB.valid())
		{
			const Body &b = constraint.bodyB.get();
			velocity = velocity - b.getVelocityAt(posB);
		}

		return velocity;
	}

	inline Vec3 getPreVelocity(const Constraint &constraint, const optional<Vec3> &posA, const optional<Vec3> &posB)
	{
		Vec3 velocity(0);
		if (constraint.bodyA.valid())
		{
			const Body &a = constraint.bodyA.get();
			velocity = a.getPreVelocityAt(posA.value());
		}

		if (constraint.bodyB.valid() && posB.has_value())
		{
			const Body &b = constraint.bodyB.get();
			velocity = velocity - b.getPreVelocityAt(posB.value());
		}

		return velocity;
	}
#pragma optimize("", off)
	void ContactConstraint::solvePositions(Constraint &constraint, const Float &dtInvSq)
	{
		auto d = constraint.getData<Data>();
		ContactPoint contact;
		const bool colliding = Collision::Penetration::compute(
			d->colliderA.get(),
			d->colliderB.get(),
			contact);

		d->colliding = colliding;
		if (!colliding)
		{
			return;
		}
		d->contact = contact;

		Vec3 posA, posB, prevA, prevB;
		getContacts(constraint, prevA, posA, prevB, posB);;

		// penetration
		Float lambdaN;
		if (constraint.computeCorrections(contact.normal, contact.depth, 0, dtInvSq, lambdaN, posA, posB))
		{
			d->force = Math::abs(lambdaN * dtInvSq);
			// apply penetration correction
			constraint.applyCorrections(contact.normal, lambdaN, false, posA, posB);
		}

		// static friction
		getContacts(constraint, posA, posB);

		const Vec3 dp = (posB - prevB) - (posA - prevA);
		const Vec3 dpTan = dp - contact.normal * dp.dot(contact.normal);
		Vec3 normalT;
		Float lambdaT;
		if (constraint.computeCorrections(dpTan, 0, dtInvSq, normalT, lambdaT, posA, posB))
		{
			if (lambdaT < d->staticFriction * lambdaN)
			{
				constraint.applyCorrections(normalT, lambdaT, false, posA, posB);
			}
		}
	}

	void ContactConstraint::solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq)
	{
		auto d = constraint.getData<Data>();
		if (d->colliding)
		{
			optional<World *> world = constraint.getWorld();
			assert(world.has_value());

			Vec3 v, posA, posB;
			Float vn;
			getContacts(constraint, posA, posB);

			// restitution
			v = getVelocity(constraint, posA, posB);
			vn = d->contact.normal.dot(v);
			const Vec3 preVel = getPreVelocity(constraint, posA, posB);

			const Float preVn = d->contact.normal.dot(preVel);
			const Float e = Math::abs(vn) < 2.0 * dt * world.value()->gravity.length() ? 0 : d->restitution;
			const Vec3 restitution = d->contact.normal * (-vn + Math::max(-e * preVn, 0));
			constraint.applyCorrections(restitution, 0, dtInvSq, true, posA, posB);

			// dynamic friction
			v = getVelocity(constraint, posA, posB);
			vn = d->contact.normal.dot(v);

			const Vec3 vt = v - d->contact.normal * vn;
			const Float vtLen = vt.length();
			const Vec3 dynamicFriction = (vt / -vtLen) * Math::min(dt * d->dynamicFriction * d->force, vtLen);

			constraint.applyCorrections(dynamicFriction, 0, dtInvSq, true, posA, posB);			
		}
	}
#pragma optimize("", on)
}