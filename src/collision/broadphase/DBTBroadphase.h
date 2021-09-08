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
			Store<Collider>::Ptr collider;
			Bounds treeBounds;

			Node() :
				collider(Store<Collider>::Ptr()),
				treeBounds(Bounds(Vec3::zero, Vec3::zero)) {}

			Node(Store<Collider>::Ptr _collider, const Bounds &_treeBounds) :
				collider(_collider),
				treeBounds(_treeBounds) {}
		};

		BoundsTree m_dynamicTree;
		BoundsTree m_staticTree;
		unordered_map<UInt32, Node> m_dynamicNodes;
		unordered_map<UInt32, Node> m_staticNodes;
		Float m_padFactor;

		UInt32 Find(const unordered_map<UInt32, Node> &nodeMap, const Store<Collider>::Ptr &collider) const;

	public:
		DBTBroadphase(const Float &padFactor = 2.0) : m_padFactor(padFactor), m_dynamicTree(BoundsTree()), m_staticTree(BoundsTree()) {}
		~DBTBroadphase() {}

#pragma region ABroadphase Interface
		virtual void add(const Store<Collider>::Ptr &collider) override;
		virtual void addStatic(const Store<Collider>::Ptr &collider) override;
		virtual void remove(const Store<Collider>::Ptr &collider) override;
		virtual void removeStatic(const Store<Collider>::Ptr &collider) override;
		virtual void update() override;

		virtual void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Store<Collider>::Ptr> &results) const override;
		virtual void generateOverlapPairs(vector<pair<Store<Collider>::Ptr, Store<Collider>::Ptr>> &results) const override;
#pragma endregion ABroadphase Interface

		void forEachNode(const function<void(Bounds)> &callback)
		{
			m_dynamicTree.forEachNode(callback);
			m_staticTree.forEachNode(callback);
		}
	};
}

#endif // DBT_BROADPHASE_H