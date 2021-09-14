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

using namespace std;

namespace Positional
{
	class World
	{
	private:
		Vec3 m_gravity;

		Store<Body> m_bodies;
		Store<Collider> m_colliders;
		Collision::ABroadphase *m_broadphase;

		Store<Collider>::Ref addCollider(const Collider &collider)
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
			return ref;
		}

	public:
		Vec3 gravity;

		World()
		{
			gravity = Vec3::zero;
			m_broadphase = new Collision::DBTBroadphase();

		}

		~World()
		{
			delete m_broadphase;
		}

		template <typename T>
		Store<Body>::Ref createBody(const Vec3 &position, const Quat &rotation)
		{
			return m_bodies.store(Body::create<T>(this, position, rotation));
		}

		void destroyBody(Store<Body>::Ref ref)
		{
			assert(ref.valid());

			m_colliders.erase([&, this](const Store<Collider>::Ref &elRef)
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

			m_bodies.erase(ref);
		}

		Store<Collider>::Ref createSphereCollider(const Store<Body>::Ref &body, const Vec3 &center, const Float &radius)
		{			
			auto collider = Collider::create<Collision::SphereCollider>(body, center, Quat::identity, Shape(radius));
			return addCollider(collider);
		}

		Store<Collider>::Ref createBoxCollider(const Store<Body>::Ref &body, const Vec3 &center, const Quat &rotation, const Vec3 &extents)
		{
			auto collider = Collider::create<Collision::BoxCollider>(body, center, rotation, Shape(extents));
			return addCollider(collider);
		}

		Store<Collider>::Ref createCapsuleCollider(Store<Body>::Ref body, const Vec3 &center, const Quat &rotation, const Float &radius, const Float &length)
		{
			auto collider = Collider::create<Collision::CapsuleCollider>(body, center, rotation, Shape(radius, length));
			return addCollider(collider);
		}

		void destroyCollider(Store<Collider>::Ref ref)
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
			m_colliders.erase(ref);
		}

		void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<RaycastResult> &results)
		{
			vector<Store<Collider>::Ref> broadResults;
			m_broadphase->raycast(ray, maxDistance, mask, broadResults);

			for (UInt32 i = 0, count = broadResults.size(); i < count; ++i)
			{
				if (broadResults[i].valid())
				{
					Vec3 point, normal;
					Float distance;
					if (broadResults[i].get().raycast(ray, maxDistance, point, normal, distance))
					{
						UInt32 idx = results.size();
						for (UInt32 j = 0, jcount = results.size(); j < jcount; ++j)
						{
							if (distance < results[j].distance)
							{
								idx = j;
								break;
							}
						}
						results.emplace(results.begin() + idx, broadResults[i], point, normal, distance);
					}
				}
			}
		}

		void updateBroadphase()
		{
			m_broadphase->update();
		}

		void forEachBoundsNode(function <void(const Bounds &bounds)> callback)
		{
			try
			{
				const auto dbt = static_cast<Collision::DBTBroadphase*>(m_broadphase);
				if (dbt)
				{
					dbt->forEachNode(callback);
				}
			}
			catch (exception exc)
			{
			}
		}

		void forEachBroadPair(function<void(const pair<Store<Collider>::Ref, Store<Collider>::Ref>&)> callback)
		{
			try
			{
				const auto dbt = static_cast<Collision::DBTBroadphase*>(m_broadphase);
				if (dbt)
				{
					vector<pair<Store<Collider>::Ref, Store<Collider>::Ref>> broadResults;
					dbt->generateOverlapPairs(broadResults);

					for (UInt32 i = 0, count = broadResults.size(); i < count; ++i)
					{
						callback(broadResults[i]);
					}
				}
			}
			catch (exception exc)
			{
			}
		}

		void forEachCollision(function<void(const CollisionResult &)> callback)
		{
			try
			{
				const auto dbt = static_cast<Collision::DBTBroadphase *>(m_broadphase);
				if (dbt)
				{
					vector<pair<Store<Collider>::Ref, Store<Collider>::Ref>> broadResults;
					dbt->generateOverlapPairs(broadResults);

					for(UInt32 i = 0, count = broadResults.size(); i < count; ++i)
					{
						ContactPoint contact;
						if (Collision::Penetration::compute(broadResults[i].first.get(), broadResults[i].second.get(), contact))
						{
							auto result = CollisionResult(broadResults[i].first, broadResults[i].second);
							result.contacts.push_back(contact);
							callback(result);
						}
					}
				}
			}
			catch (exception exc)
			{
			}
		}
	};
}
#endif // WORLD_H