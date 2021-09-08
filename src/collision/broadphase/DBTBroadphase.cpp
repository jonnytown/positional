#include "DBTBroadphase.h"

namespace Positional::Collision
{
#pragma region ABroadphase Interface
	void DBTBroadphase::add(const Store<Collider>::Ptr &ptr)
	{
		const Collider *collider = ptr.get();
		const Bounds &bounds = collider->bounds();
		Bounds treeBounds = Bounds(bounds.center, bounds.extents() * m_padFactor);
		UInt32 handle = m_dynamicTree.add(treeBounds, collider->mask);
		m_dynamicNodes[handle] = Node(ptr, treeBounds);
	}

	void DBTBroadphase::addStatic(const Store<Collider>::Ptr &ptr)
	{
		const Collider *collider = ptr.get();
		const Bounds &bounds = collider->bounds();
		UInt32 handle = m_staticTree.add(bounds, collider->mask);
		m_staticNodes[handle] = Node(ptr, bounds);
	}

	void DBTBroadphase::remove(const Store<Collider>::Ptr &ptr)
	{
		UInt32 handle = Find(m_dynamicNodes, ptr);
		assert(handle != NOT_FOUND);

		m_dynamicTree.remove(handle);
		m_dynamicNodes.erase(handle);
	}

	void DBTBroadphase::removeStatic(const Store<Collider>::Ptr &ptr)
	{
		UInt32 handle = Find(m_staticNodes, ptr);
		assert(handle != NOT_FOUND);

		m_staticTree.remove(handle);
		m_staticNodes.erase(handle);
	}

	void DBTBroadphase::update()
	{
		for (auto &[handle, node] : m_dynamicNodes)
		{
			const Collider *collider = node.collider.get();
			const Bounds &bounds = collider->bounds();

			if (!node.treeBounds.contains(bounds))
			{
				node.treeBounds = Bounds(bounds.center, bounds.extents() * m_padFactor);
				m_dynamicTree.update(handle, node.treeBounds, collider->mask);
			}
		}

		// TODO: update static tree only when it is dirty
		for (auto &[handle, node] : m_staticNodes)
		{
			const Collider *collider = node.collider.get();
			const Bounds &bounds = collider->bounds();

			if (!node.treeBounds.contains(bounds))
			{
				node.treeBounds = Bounds(bounds.center, bounds.extents());
				m_staticTree.update(handle, node.treeBounds, collider->mask);
			}
		}
	}

	void DBTBroadphase::raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Store<Collider>::Ptr> &results) const
	{
		m_dynamicTree.raycast(
			ray,
			maxDistance,
			mask,
			[&, this](const UInt32 &handle)
			{
				const Node &node = m_dynamicNodes.at(handle);
				results.push_back(node.collider);
			});

		m_staticTree.raycast(
			ray,
			maxDistance,
			mask,
			[&, this](const UInt32 &handle)
			{
				const Node &node = m_staticNodes.at(handle);
				results.push_back(node.collider);
			});
	}

	void DBTBroadphase::generateOverlapPairs(vector<pair<Store<Collider>::Ptr, Store<Collider>::Ptr>> &results) const
	{
		m_dynamicTree.generateOverlapPairs(
			[&, this](const auto &pair)
			{
				const Node &node1 = m_dynamicNodes.at(pair.first);
				const Node &node2 = m_dynamicNodes.at(pair.second);
				// TODO: find better way to check for colliders with the same body, possibly need to implement compound collider
				if (node1.collider.get()->body() != node2.collider.get()->body())
				{
					results.push_back(make_pair(node1.collider, node2.collider));
				}
			},
			false);

		for (const auto &[handle, node] : m_dynamicNodes)
		{
			auto ptr = node.collider;
			const Collider *collider = ptr.get();
			m_staticTree.intersects(
				collider->bounds(),
				collider->mask,
				[&, this](const UInt32 &handle)
				{
					const Node &staticNode = m_staticNodes.at(handle);
					results.push_back(make_pair(ptr, staticNode.collider));
				});
		}
	}

	UInt32 DBTBroadphase::Find(const unordered_map<UInt32, Node> &nodeMap, const Store<Collider>::Ptr &collider) const
	{
		for (const auto &[key, node] : nodeMap)
		{
			if (node.collider == collider)
			{
				return key;
			}
		}
		return NOT_FOUND;
	}
#pragma endregion ABroadphase Interface
}