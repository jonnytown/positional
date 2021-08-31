/*
 * Dynamic Bounds Tree Broadphase implementation (dynamic volume bounds hierarchy)
 */
#ifndef DBT_BROADPHASE_H
#define DBT_BROADPHASE_H

#include "ABroadphase.h"
#include "BoundsTree.h"
#include "math/Math.h"
#include <optional>

using namespace std;

namespace Positional::Collision
{
	class DBTBroadphase : public ABroadphase
	{
	private:
		class Node
		{
		public:
			optional<Collider*> collider;
			Bounds treeBounds;

			Node() {}
			Node(Collider *_collider, const Bounds &_treeBounds) : treeBounds(_treeBounds)
			{
				collider.emplace(_collider);
			}
		};

		BoundsTree m_dynamicTree;
		BoundsTree m_staticTree;
		unordered_map<UInt32, Node> m_dynamicNodes;
		unordered_map<UInt32, Node> m_staticNodes;
		Float m_padFactor;

	public:
		DBTBroadphase(const Float &padFactor = 2.0) : m_padFactor(padFactor) {}
		~DBTBroadphase(){}

#pragma region ABroadphase Interface
		virtual UInt32 add(Collider *collider) override;
		virtual UInt32 addStatic(Collider *collider) override;
		virtual void remove(const UInt32 &handle) override;
		virtual void removeStatic(const UInt32 &handle) override;
		virtual void update() override;

		virtual void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Collider *> &results) const override;
		virtual void generateOverlapPairs(vector<pair<Collider *, Collider *>> &results) const override;
#pragma endregion ABroadphase Interface
	};
}

#endif // DBT_BROADPHASE_H