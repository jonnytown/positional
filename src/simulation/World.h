#ifndef WORLD_H
#define WORLD_H

#include "math/Math.h"
#include "Body.h"
#include "collision/collider/SphereCollider.h"
#include "collision/collider/BoxCollider.h"
#include "collision/collider/CapsuleCollider.h"
#include <unordered_set>
#include "data/Store.h"
#include "collision/broadphase/DBTBroadphase.h"
#include "collision/narrowphase/Penetration.h"
#include "collision/narrowphase/RaycastResult.h"
#include "collision/narrowphase/CollisionResult.h"
#include "constraints/ContactConstraint.h"

using namespace std;

namespace Positional
{
	class World
	{
	private:
		Store<Body> m_bodies;
		Store<Collider> m_colliders;
		Collision::ABroadphase *m_broadphase;

		Ref<Collider> addCollider(const Ref<Body> &body, const Collider &collider);

		vector<Constraint> m_contacts;
		UInt32 m_contactCount;
	public:
		Vec3 gravity;

		World();
		~World();

		template <typename T>
		Ref<Body> createBody(const Vec3 &position, const Quat &rotation)
		{
			return m_bodies.store(Body::create<T>(this, position, rotation));
		}
		void destroyBody(Ref<Body> ref);

		Ref<Collider> createSphereCollider(const Ref<Body> &body, const Vec3 &center, const Float &radius, const Float &density, const Float &staticFriction, const Float &dynamicFriction, const Float &bounciness);
		Ref<Collider> createBoxCollider(const Ref<Body> &body, const Vec3 &center, const Quat &rotation, const Vec3 &extents, const Float &density, const Float &staticFriction, const Float &dynamicFriction, const Float &bounciness);
		Ref<Collider> createCapsuleCollider(const Ref<Body> &body, const Vec3 &center, const Quat &rotation, const Float &radius, const Float &length, const Float &density, const Float &staticFriction, const Float &dynamicFriction, const Float &bounciness);
		void destroyCollider(Ref<Collider> ref);

		void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<RaycastResult> &results) const;
		void forEachBody(const function<void(const Ref<Body> &)> &callback);
		void forEachBoundsNode(const function <void(const Bounds &bounds)> &callback) const;
		void forEachBroadPair(const function<void(const pair<Ref<Collider>, Ref<Collider>>&)> &callback) const;
		void forEachCollision(const function<void(const CollisionResult &)> &callback) const;

		void updateBroadphase();
		void simulate(const Float &deltaTime, const UInt32 &subSteps);
	};
}
#endif // WORLD_H