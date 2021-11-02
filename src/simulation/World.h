#ifndef WORLD_H
#define WORLD_H

#include "math/Math.h"
#include "Body.h"
#include "collision/collider/SphereCollider.h"
#include "collision/collider/BoxCollider.h"
#include "collision/collider/CapsuleCollider.h"
#include <unordered_set>
#include "data/Store.h"
#include "data/IdPair.h"
#include "collision/broadphase/IBroadphase.h"
#include "collision/narrowphase/INarrowphase.h"
#include "collision/narrowphase/RaycastResult.h"
#include "collision/narrowphase/CollisionResult.h"
#include "constraints/Constraint.h"

using namespace std;

namespace Positional
{
	typedef function<void(const RaycastResult &)> RaycastCallback;
	typedef function<void(const CollisionResult &)> CollisionCallback;
	typedef function<void(const Ref<Body> &)> BodyCallback;

	class World
	{
	private:
		Store<Body> m_bodies;
		Store<Collider> m_colliders;
		Store<Constraint> m_constraints;
		Collision::IBroadphase *m_broadphase;
		Collision::INarrowphase *m_narrowphase;

		unordered_set<IdPair<UInt64>, IdPair<UInt64>::SYM_HASH, IdPair<UInt64>::SYM_EQ> m_ignoreBodies;
		unordered_set<IdPair<UInt64>, IdPair<UInt64>::SYM_HASH, IdPair<UInt64>::SYM_EQ> m_ignoreColliders;

		vector<Constraint> m_contacts;
		UInt32 m_contactCount;

		Ref<Collider> addCollider(const Ref<Body> &body, const Collider &collider);
	public:
		Vec3 gravity;

		World();
		~World();

		template <class T>
		Ref<Body> createBody(const Vec3 &position, const Quat &rotation)
		{
			return m_bodies.store(Body::create<T>(this, position, rotation));
		}
		void destroyBody(Ref<Body> ref);

		template <class ColliderT, typename... ShapeArgs>
		Ref<Collider> createCollider(const Ref<Body> &body, const Vec3 &position, const Quat &rotation, const Float &density, const Float &staticFriction, const Float &dynamicFriction, const Float &bounciness, ShapeArgs &&...shapeArgs)
		{
			auto collider = Collider::create<ColliderT>(body, position, rotation, Shape(shapeArgs...), density, staticFriction, dynamicFriction, bounciness);
			return addCollider(body, collider);
		}
		
		void destroyCollider(Ref<Collider> ref);

		template <class ConstraintT, class DataT, typename... DataArgs>
		Ref<Constraint> createConstraint(const Ref<Body> &bodyA, const Ref<Body> &bodyB, const bool &ignoreCollisions, DataArgs &&...dataArgs)
		{
			Constraint joint = Constraint::create<ConstraintT, DataT>();
			joint.init<DataT>(bodyA, bodyB, ignoreCollisions, dataArgs...);
			return m_constraints.store(joint);
		}

		void destroyConstraint(Ref<Constraint> ref)
		{
			m_constraints.erase(ref);
		}

		void raycast(const Ray &ray, const UInt32 &mask, const Float &maxDistance, const RaycastCallback &callback) const;
		void forEachBody(const BodyCallback &callback);
		void forEachBoundsNode(const function <void(const Bounds &bounds)> &callback) const;
		void forEachBroadPair(const Collision::OverlapCallback &callback) const;
		void forEachCollision(const CollisionCallback &callback) const;

		void updateBroadphase();
		void simulate(const Float &deltaTime, const UInt32 &subSteps);
	};
}
#endif // WORLD_H