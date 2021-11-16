#include "World.h"
#include "collision/broadphase/DBTBroadphase.h"
#include "collision/narrowphase/GJKEPANarrowphase.h"
#include "constraints/ContactConstraint.h"

namespace Positional
{
	World::World()
	{
		m_contactCount = 0;
		gravity = Vec3::zero;
		m_broadphase = new Collision::DBTBroadphase(2.0);
		m_narrowphase = new Collision::GJKEPANarrowphase();
	}

	World::~World()
	{
		delete m_broadphase;
		delete m_narrowphase;
	}

#pragma region Bodies

	void World::destroyBody(Ref<Body> ref)
	{
		assert(ref.valid());

		m_colliders.erase([&, this](const Ref<Collider> &elRef)
		{
			const Collider &collider = elRef.get();
			if (collider.body() == ref)
			{
				if (collider.isStatic())
				{
					m_broadphase->removeStatic(elRef);
				}
				else
				{
					m_broadphase->remove(elRef);
				}
				return true;
			}
			return false;
		});

		m_constraints.erase([&, this](const Ref<Constraint> &elRef)
		{
			const Constraint &constraint = elRef.get();
			if ((constraint.bodyA == ref && !constraint.bodyB.valid()) || (constraint.bodyB == ref && !constraint.bodyA.valid()))
			{
				return true;
			}
			return false;
		});

		m_bodies.erase(ref);
	}
#pragma endregion // Bodies

#pragma region Colliders
	Ref<Collider> World::addCollider(const Ref<Body> &bodyRef, const Collider &collider)
	{
		auto ref = m_colliders.store(collider);
		if (collider.isStatic())
		{
			m_broadphase->addStatic(ref);
		}
		else
		{
			m_broadphase->add(ref);
		}

		if (bodyRef.valid())
		{
			Body &body = bodyRef.get();
			body.m_colliders.push_back(ref);
			body.updateMass();
		}

		return ref;
	}

	void World::destroyCollider(Ref<Collider> ref)
	{
		assert(ref.valid());
		if (ref.get().isStatic())
		{
			m_broadphase->removeStatic(ref);
		}
		else
		{
			m_broadphase->remove(ref);
		}

		// erase collider from body entry
		if (ref.get().body().valid())
		{
			auto &bodyColliders = ref.get().body().get().m_colliders;
			for (UInt32 i = 0, count = bodyColliders.size(); i < count; ++i)
			{
				if (bodyColliders[i] == ref)
				{
					bodyColliders.erase(bodyColliders.begin() + i);
					break;
				}
			}
		}

		// erase from store
		m_colliders.erase(ref);
	}
#pragma endregion // Colliders

#pragma region Queries
	void World::raycast(const Ray &ray, const UInt32 &mask, const Float &maxDistance, const RaycastCallback &callback) const
	{
		m_broadphase->raycast(ray, mask, maxDistance, [&](Ref<Collider> ref) {
			// Vec3 point, normal;
			// Float distance;
			RaycastResult result;
			if (ref.get().raycast(ray, maxDistance, result.point, result.normal, result.distance))
			{
				result.collider = ref;
				callback(result);
			}
		});
	}

	void World::forEachBody(const function<void(const Ref<Body> &)> &callback)
	{
		m_bodies.forEach(callback);
	}

	void World::forEachBoundsNode(const function<void(const Bounds &bounds)> &callback) const
	{
		const auto dbt = static_cast<Collision::DBTBroadphase *>(m_broadphase);
		if (dbt)
		{
			dbt->forEachNode(callback);
		}
	}

	void World::forEachBroadPair(const Collision::OverlapCallback &callback) const
	{
		m_broadphase->forEachOverlapPair(callback);
	}

	void World::forEachCollision(const function<void(const CollisionResult &)> &callback) const
	{
		if (m_contactCount > 0)
		{
			for (UInt32 i = 0; i < m_contactCount; ++i)
			{
				auto d = m_contacts[i].getData<ContactConstraint::Data>();
				if (d->colliding)
				{
					auto result = CollisionResult(d->colliderA, d->colliderB, d->contact);
					callback(result);
				}
			}
		}
		else
		{
			m_broadphase->forEachOverlapPair([&](const pair<Ref<Collider>, Ref<Collider>> &pair)
			{
				ContactPoint contact;
				if (m_narrowphase->compute(pair.first.get(), pair.second.get(), contact))
				{
					auto result = CollisionResult(pair.first, pair.second, contact);
					callback(result);
				}
			});
		}
	}
#pragma endregion // Queries

#pragma region Simulation
	void World::updateBroadphase()
	{
		m_broadphase->update(0);
	}

	void World::simulate(const Float &deltaTime, const UInt32 &_subSteps)
	{
		const UInt32 subSteps = Math::max(_subSteps, 1);
		const Float h = deltaTime / subSteps;
		const Float hInv = 1.0/h;
		const Float hInvSq = hInv*hInv;

		// collect collision pairs
		m_contactCount = 0;
		m_broadphase->update(deltaTime);
		m_broadphase->forEachOverlapPair([&, this](const pair<Ref<Collider>, Ref<Collider>> &pair)
		{	
			const Ref<Body> &bodyA = pair.first.get().body();
			const Ref<Body> &bodyB = pair.second.get().body();
			bool ignoreCollisions = false;
			m_constraints.first([&](const Ref<Constraint> &ref)
			{	
				const Constraint &constraint = ref.get();
				if ((constraint.bodyA == bodyA && constraint.bodyB == bodyB) || (constraint.bodyA == bodyB && constraint.bodyB == bodyA))
				{
					ignoreCollisions = constraint.ignoreCollisions;
					return true;
				}
				return false;
			});

			if (ignoreCollisions)
			{
				return;
			}

			if (m_contactCount >= m_contacts.size())
			{
				m_contacts.push_back(Constraint::create<ContactConstraint, ContactConstraint::Data>());
			}

			m_contacts[m_contactCount].init<ContactConstraint::Data>(
				bodyA,
				bodyB,
				false,
				pair.first,
				pair.second,
				m_narrowphase
			);

			m_contactCount++;
		});

		for (UInt32 s = 0; s < subSteps; ++s)
		{
			// constraint fores
			for (UInt32 i = 0, count = m_constraints.count(); i < count; ++i)
			{
				m_constraints[i].applyForces(h);
			}

			// integrate
			for (UInt32 i = 0, count = m_bodies.count(); i < count; ++i)
			{
				m_bodies[i].integrate(h, gravity);
			}

			// solve positions for each constraint
			for (UInt32 i = 0, count = m_constraints.count(); i < count; ++i)
			{
				m_constraints[i].solvePositions(hInvSq);
			}

			
			for (UInt32 i = 0; i < m_contactCount; ++i)
			{
				m_contacts[i].solvePositions(hInvSq);
			}

			// differentiate
			for (UInt32 i = 0, count = m_bodies.count(); i < count; ++i)
			{
				m_bodies[i].differentiate(hInv);
			}

			// solve velocities for each constraint
			for (UInt32 i = 0; i < m_contactCount; ++i)
			{
				m_contacts[i].solveVelocities(h, hInvSq);
			}

			for (UInt32 i = 0, count = m_constraints.count(); i < count; ++i)
			{
				m_constraints[i].solveVelocities(h, hInvSq);
			}
		}
	}

#pragma endregion // Simulation
	
} // namespace Positional