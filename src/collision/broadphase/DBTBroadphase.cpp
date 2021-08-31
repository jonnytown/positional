#include "DBTBroadphase.h"

namespace Positional::Collision
{
#pragma region ABroadphase Interface
	UInt32 DBTBroadphase::add(Collider *collider)
	{
		const Bounds &bounds = collider->bounds();
		Bounds treeBounds = Bounds(bounds.center, bounds.extents() * m_padFactor);
		UInt32 handle = m_dynamicTree.add(treeBounds, collider->mask);
		m_dynamicNodes[handle] = Node(collider, treeBounds);
		return handle;
	}

	UInt32 DBTBroadphase::addStatic(Collider *collider)
	{
		const Bounds &bounds = collider->bounds();
		UInt32 handle = m_staticTree.add(bounds, collider->mask);
		m_staticNodes[handle] = Node(collider, bounds);
		return handle;
	}

	void DBTBroadphase::remove(const UInt32 &handle)
	{
		assert (m_dynamicNodes.count(handle));
		m_dynamicTree.remove(handle);
		m_dynamicNodes.erase(handle);
	}

	void DBTBroadphase::removeStatic(const UInt32 &handle)
	{
		assert(m_staticNodes.count(handle));
		m_staticTree.remove(handle);
		m_staticNodes.erase(handle);
	}

	void DBTBroadphase::update()
	{
		for (auto &[handle, node] : m_dynamicNodes)
		{
			const Bounds &bounds = node.collider.value()->bounds();

			if (!node.treeBounds.contains(bounds))
			{
				node.treeBounds = Bounds(bounds.center, bounds.extents() * m_padFactor);
				m_dynamicTree.update(handle, node.treeBounds, node.collider.value()->mask);
			}
		}

		// TODO: update static tree only when it is dirty
		for (auto &[handle, node] : m_staticNodes)
		{
			const Bounds &bounds = node.collider.value()->bounds();

			if (!node.treeBounds.contains(bounds))
			{
				node.treeBounds = Bounds(bounds.center, bounds.extents() * m_padFactor);
				m_staticTree.update(handle, node.treeBounds, node.collider.value()->mask);
			}
		}
	}

	void DBTBroadphase::raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Collider *> &results) const
	{
		m_dynamicTree.raycast(
			ray,
			maxDistance,
			mask,
			[&, this](const UInt32 &result)
			{
				const Node &node = m_dynamicNodes.at(result);
				results.push_back(node.collider.value());
			}
		);

		m_staticTree.raycast(
			ray,
			maxDistance,
			mask,
			[&, this](const UInt32 &result)
			{
				const Node &node = m_staticNodes.at(result);
				results.push_back(node.collider.value());
			});
	}

	void DBTBroadphase::generateOverlapPairs(vector<pair<Collider *, Collider *>> &results) const
	{
		m_dynamicTree.generateOverlapPairs(
			[&, this](const auto &pair)
			{
				const Node &node1 = m_dynamicNodes.at(pair.first);
				const Node &node2 = m_dynamicNodes.at(pair.second);

				if (node1.collider.value()->body().value() != node2.collider.value()->body().value())
				{
					results.push_back(make_pair(node1.collider.value(), node2.collider.value()));
				}
			},
			false
		);

		for (const auto &[handle, node] : m_dynamicNodes)
		{
			Collider *collider = node.collider.value();
			m_staticTree.intersects(
				collider->bounds(),
				collider->mask,
				[&, this](const UInt32 &result)
				{
					const Node &staticNode = m_staticNodes.at(result);
					results.push_back(make_pair(collider, staticNode.collider.value()));
				}
			);
		}
	}
#pragma endregion ABroadphase Interface
}