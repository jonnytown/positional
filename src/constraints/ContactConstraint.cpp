#include "ContactConstraint.h"

#include "collision/narrowphase/Penetration.h"
#include "simulation/World.h"

namespace Positional
{
	inline void getContacts(const Constraint &constraint, const ContactConstraint::Data *data, Vec3 &posA, Vec3 &posB)
	{
		posA = Body::pointToWorld(constraint.bodyA, data->contact.pointA);
		posB = Body::pointToWorld(constraint.bodyB, data->contact.pointB);
	}

	inline void getPreContacts(const Constraint &constraint, const ContactConstraint::Data *data, Vec3 &posA, Vec3 &posB)
	{
		posA = Body::prePointToWorld(constraint.bodyA, data->contact.pointA);
		posB = Body::prePointToWorld(constraint.bodyB, data->contact.pointB);
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
		auto data = constraint.getData<Data>();
		data->update();
		if (!data->colliding)
		{
			return;
		}

		Vec3 posA, posB;
		getContacts(constraint, data, posA, posB);
		// penetration
		Float lambdaN;
		if (constraint.computeCorrections(data->contact.normal, data->contact.depth, 0, dtInvSq, lambdaN, posA, posB))
		{
			data->contact.force = Math::abs(lambdaN * dtInvSq);
			// apply penetration correction
			constraint.applyCorrections(data->contact.normal, lambdaN, false, posA, posB);

			// static friction
			Vec3 preA, preB;
			getContacts(constraint, data, posA, posB);
			getPreContacts(constraint, data, preA, preB);

			const Vec3 dp = (posB - preB) - (posA - preA);
			const Vec3 dpTan = dp - data->contact.normal * dp.dot(data->contact.normal);
			Vec3 normalT;
			Float lambdaT;
			if (constraint.computeCorrections(dpTan, 0, dtInvSq, normalT, lambdaT, posA, posB) &&
				Math::abs(lambdaT) > Math::abs(data->staticFriction * lambdaN))
			{
				constraint.applyCorrections(normalT, lambdaT, false, posA, posB);
			}
		}
	}

	void ContactConstraint::solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq)
	{
		auto data = constraint.getData<Data>();
		if (data->colliding)
		{
			optional<World *> world = constraint.getWorld();
			assert(world.has_value());

			Vec3 v, preA, posA, preB, posB;
			Float vn;
			getPreContacts(constraint, data, preA, preB);
			getContacts(constraint, data, posA, posB);

			v = getVelocity(constraint, posA, posB);
			vn = data->contact.normal.dot(v);

			// dynamic friction
			const Vec3 vt = v - data->contact.normal * vn;
			const Float vtLen = vt.length();
			const Vec3 dynamicFriction = vt * -(Math::min(dt * data->dynamicFriction * data->contact.force, vtLen) / vtLen);

			constraint.applyCorrections(dynamicFriction, 0, dtInvSq, true, posA, posB);

			// restitution
			v = getVelocity(constraint, posA, posB);
			vn = data->contact.normal.dot(v);
			const Vec3 preVel = getPreVelocity(constraint, preA, preB);

			const Float preVn = data->contact.normal.dot(preVel);
			const Float e = Math::abs(vn) < 2.0 * dt * world.value()->gravity.length() ? 0 : data->restitution;
			const Vec3 restitution = data->contact.normal * (-vn + Math::max(-e * preVn, 0));
			constraint.applyCorrections(restitution, 0, dtInvSq, true, posA, posB);
		}
	}
}