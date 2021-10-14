#include "ContactConstraint.h"

#include "collision/narrowphase/Penetration.h"
#include "simulation/World.h"

namespace Positional
{

	inline void getContacts(const Constraint &constraint, const ContactConstraint::Data *d, Vec3 &posA, Vec3 &posB)
	{
		posA = Body::pointToWorld(constraint.bodyA, d->contact.pointA);
		posB = Body::pointToWorld(constraint.bodyB, d->contact.pointB);
	}

	inline void getPreContacts(const Constraint &constraint, const ContactConstraint::Data *d, Vec3 &posA, Vec3 &posB)
	{
		posA = Body::prePointToWorld(constraint.bodyA, d->contact.pointA);
		posB = Body::prePointToWorld(constraint.bodyB, d->contact.pointB);
	}


	inline void getCOMs(const Constraint &constraint, Vec3 &preA, Vec3 &comA, Vec3 &preB, Vec3 &comB)
	{
		preA = Body::preCOM(constraint.bodyA);
		comA = Body::COM(constraint.bodyA);
		preB = Body::preCOM(constraint.bodyB);
		comB = Body::COM(constraint.bodyB);
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

	inline Vec3 getPreVelocity(const Constraint &constraint, const Vec3 &posA, const Vec3 &posB)
	{
		Vec3 velocity(0);
		if (constraint.bodyA.valid())
		{
			const Body &a = constraint.bodyA.get();
			velocity = a.getPreVelocityAt(posA);
		}

		if (constraint.bodyB.valid())
		{
			const Body &b = constraint.bodyB.get();
			velocity = velocity - b.getPreVelocityAt(posB);
		}

		return velocity;
	}

		void ContactConstraint::solvePositions(Constraint &constraint, const Float &dtInvSq)
	{
		auto d = constraint.getData<Data>();
		d->update();
		if (!d->colliding)
		{
			return;
		}

		Vec3 posA, posB;
		getContacts(constraint, d, posA, posB);
		// penetration
		Float lambdaN;
		if (constraint.computeCorrections(d->contact.normal, d->contact.depth, 0, dtInvSq, lambdaN, posA, posB))
		{
			d->contact.force = Math::abs(lambdaN * dtInvSq);
			// apply penetration correction
			constraint.applyCorrections(d->contact.normal, lambdaN, false, posA, posB);
		}

		// static friction
		Vec3 preA, preB;
		getContacts(constraint, d, posA, posB);
		getPreContacts(constraint, d, preA, preB);

		const Vec3 dp = (posB - preB) - (posA - preA);
		const Vec3 dpTan = dp - d->contact.normal * dp.dot(d->contact.normal);
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

			Vec3 v, preA, posA, preB, posB;
			Float vn;
			getPreContacts(constraint, d, preA, preB);
			getContacts(constraint, d, posA, posB);

			// restitution
			v = getVelocity(constraint, posA, posB);
			vn = d->contact.normal.dot(v);
			const Vec3 preVel = getPreVelocity(constraint, preA, preB);

			const Float preVn = d->contact.normal.dot(preVel);
			const Float e = Math::abs(vn) < 2.0 * dt * world.value()->gravity.length() ? 0 : d->restitution;
			const Vec3 restitution = d->contact.normal * (-vn + Math::max(-e * preVn, 0));
			constraint.applyCorrections(restitution, 0, dtInvSq, true, posA, posB);

			// dynamic friction
			v = getVelocity(constraint, posA, posB);
			vn = d->contact.normal.dot(v);

			const Vec3 vt = v - d->contact.normal * vn;
			const Float vtLen = vt.length();
			const Vec3 dynamicFriction = (vt / -vtLen) * Math::min(dt * d->dynamicFriction * d->contact.force, vtLen);

			constraint.applyCorrections(dynamicFriction, 0, dtInvSq, true, posA, posB);
		}
	}
}