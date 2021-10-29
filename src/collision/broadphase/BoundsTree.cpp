#include "BoundsTree.h"
#include <deque>

namespace Positional::Collision
{
#pragma region Public
	UInt32 BoundsTree::add(const Bounds &bounds, const UInt32 &mask)
	{
		UInt32 handle = nextHandle();
		add(bounds, mask, handle);
		m_leaves.insert(handle);
		return handle;
	}

	void BoundsTree::update(const UInt32 &handle, const Bounds &bounds, const UInt32 &mask)
	{
		assert(m_nodes.count(handle) && m_nodes.at(handle).isLeaf());
		remove(handle, false);
		add(bounds, mask, handle);
	}

	void BoundsTree::updateMask(const UInt32 &handle, const UInt32 &mask)
	{
		assert(m_nodes.count(handle));
		Node &node = m_nodes.at(handle);
		assert(node.isLeaf());

		node.mask = mask;

		UInt32 ancestorHandle = node.parent;
		while (ancestorHandle != NOT_FOUND)
		{
			Node &ancestor = m_nodes.at(ancestorHandle);
			ancestor.mask = m_nodes.at(ancestor.children[0]).mask | m_nodes.at(ancestor.children[0]).mask;
			ancestorHandle = ancestor.parent;
		}
	}

	void BoundsTree::remove(const UInt32 &handle)
	{
		assert(m_nodes.count(handle));
		remove(handle, true);
		m_leaves.erase(handle);
	}

	void BoundsTree::raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, const ResultCallback &resultsCallback) const
	{
		if (m_root == NOT_FOUND)
		{
			return;
		}

		deque<UInt32> stack;
		stack.push_back(m_root);

		while (stack.size() > 0)
		{
			const UInt32 handle = stack.back();
			stack.pop_back();

			const Node &node = m_nodes.at(handle);
			Vec3 point, normal;
			Float distance;
			if ((mask & node.mask) != 0
				&& node.bounds.intersects(ray, distance)
				&& (maxDistance <= 0 || distance <= maxDistance))
			{
				if (node.isLeaf())
				{
					resultsCallback(handle);
				}
				else
				{
					stack.push_back(node.children[1]);
					stack.push_back(node.children[0]);
				}
			}
		}
	}

	void BoundsTree::intersects(const Bounds &bounds, const UInt32 &mask, const ResultCallback &resultsCallback, const bool &exclusive) const
	{
		if (m_root == NOT_FOUND)
		{
			return;
		}

		deque<UInt32> stack;
		stack.push_back(m_root);

		while (stack.size() > 0)
		{
			const UInt32 handle = stack.back();
			stack.pop_back();

			const Node &node = m_nodes.at(handle);
			if ((mask & node.mask) != 0 && node.bounds.intersects(bounds, exclusive))
			{
				if (node.isLeaf())
				{
					resultsCallback(handle);
				}
				else
				{
					stack.push_back(node.children[1]);
					stack.push_back(node.children[0]);
				}
			}
		}
	}

	void BoundsTree::forEachOverlapPair(const ResultPairCallback &resultsCallback, const bool &exclusive) const
	{
		if (m_root == NOT_FOUND)
		{
			return;
		}

		unordered_set<HandlePair, HandlePair::SYM_HASH, HandlePair::SYM_EQ> pairs;

		for (const UInt32 &leafHandle : m_leaves)
		{
			deque<UInt32> stack;
			stack.push_back(m_root);

			const Node &leaf = m_nodes.at(leafHandle);
			const UInt32 mask = leaf.mask;
			const Bounds &bounds = leaf.bounds;

			while (stack.size() > 0)
			{
				const UInt32 handle = stack.back();
				stack.pop_back();

				if (handle != leafHandle)
				{
					const Node &node = m_nodes.at(handle);
					const HandlePair pair(leafHandle, handle);
					if ((mask & node.mask) != 0 && pairs.count(pair) == 0 && node.bounds.intersects(bounds, exclusive))
					{
						if (node.isLeaf())
						{
							pairs.insert(pair);
							resultsCallback(pair);
						}
						else
						{
							stack.push_back(node.children[1]);
							stack.push_back(node.children[0]);
						}
					}
				}
			}
		}
	}
#pragma endregion // Public

#pragma region Private
	void
	BoundsTree::add(const Bounds &bounds, const UInt32 &mask, const UInt32 &handle)
	{
		UInt32 parentHandle = NOT_FOUND, oldParentHandle = NOT_FOUND;

		if (m_nodes.size() == 0)
		{
			m_root = handle;
			Node node(bounds, mask, parentHandle);
			m_nodes[handle] = node;
			return;
		}

		// find best sibling
		UInt32 sibHandle = findBestSibling(bounds);
		assert(sibHandle != NOT_FOUND);
		Node &sibling = m_nodes.at(sibHandle);

		// create new parent
		oldParentHandle = sibling.parent;
		parentHandle = nextHandle();
		Node parent(oldParentHandle);
		

		if (oldParentHandle == NOT_FOUND)
		{
			// sibling was root
			m_root = parentHandle;
		}
		else
		{
			// sibling was not root
			Node &oldParent = m_nodes.at(oldParentHandle);
			UInt32 childIdx = oldParent.children[1] == sibHandle;
			oldParent.children[childIdx] = parentHandle;
		}

		parent.children[0] = sibHandle;
		parent.children[1] = handle;
		sibling.parent = parentHandle;

		Node node(bounds, mask, parentHandle);
		m_nodes[parentHandle] = parent;
		m_nodes[handle] = node;

		refit(handle);
	}

	void BoundsTree::remove(const UInt32 &handle, const bool &refitAncestors)
	{
		const Node &node = m_nodes.at(handle);
		assert(node.isLeaf());

		if (node.parent != NOT_FOUND)
		{
			const Node &parent = m_nodes.at(node.parent);

			UInt32 childIdx = parent.children[1] == handle;
			UInt32 sibHandle = parent.children[1 - childIdx];
			Node &sibling = m_nodes.at(sibHandle);

			sibling.parent = parent.parent;
			if (sibling.parent == NOT_FOUND)
			{
				m_root = sibHandle;
			}
			else
			{
				Node &newParent = m_nodes.at(sibling.parent);
				UInt32 newSibIdx = newParent.children[1] == node.parent;
				newParent.children[newSibIdx] = sibHandle;

				if (refitAncestors)
				{
					refit(sibHandle);
				}
			}

			m_nodes.erase(node.parent);

			if (m_root == handle)
			{
				m_root = NOT_FOUND;
			}
		}

		m_nodes.erase(handle);
	}

	/*
	 * Insertion cost is calculated as surface area (bounds u node) + delta surface area (bounds u ancestor) for each ancestor
	 * see https://box2d.org/files/ErinCatto_DynamicBVH_GDC2019.pdf SAH insertion cost
	 */
	UInt32 BoundsTree::findBestSibling(const Bounds &bounds) const
	{
		assert(m_nodes.size() >= 3);

		Float bestCost = FLOAT_MAX;
		UInt32 bestHandle = NOT_FOUND;

		const Float sa = bounds.surfaceArea();

		deque<tuple<UInt32, Float>> queue;
		// node, inhereted
		queue.push_front(make_tuple(m_root, 0));

		while (queue.size() > 0)
		{
			const auto &[handle, inheritedCost] = queue.back();
			queue.pop_back();

			if (sa + inheritedCost < bestCost) // low bounds test
			{
				const Node& node = m_nodes.at(handle);
				const Float directCost = node.bounds.merged(bounds).surfaceArea();

				const Float cost = directCost + inheritedCost;
				if (cost < bestCost)
				{
					if (node.isLeaf())
					{
						bestCost = cost;
						bestHandle = handle;
					}
					else
					{
						const Float deltaCost = directCost - node.bounds.surfaceArea();
						const Float nextInherited = inheritedCost + deltaCost;
						queue.push_back(make_tuple(node.children[0], nextInherited));
						queue.push_back(make_tuple(node.children[1], nextInherited));
					}
				}
			}
		}

		return bestHandle;
	}

	void BoundsTree::refit(const UInt32 &startHandle)
	{
		const Node &startNode = m_nodes.at(startHandle);

		UInt32 handle = startHandle;
		Bounds nextBounds = startNode.isLeaf() ? Bounds(Vec3::zero, Vec3::zero) : m_nodes.at(startNode.children[0]).bounds.merged(m_nodes.at(startNode.children[1]).bounds);

		while (handle != NOT_FOUND)
		{
			Node& node = m_nodes.at(handle);

			if (!node.isLeaf())
			{
				node.bounds = nextBounds;
				node.mask = m_nodes.at(node.children[0]).mask | m_nodes.at(node.children[1]).mask;
			}

			const UInt32 parentHandle = node.parent;
			if (parentHandle != NOT_FOUND)
			{
				Node &parent = m_nodes.at(parentHandle);

				const UInt32 sibIdx = 1 - (parent.children[1] == handle);
				Node &sibling = m_nodes.at(parent.children[sibIdx]);
				nextBounds = node.bounds.merged(sibling.bounds);

				// try to rotate
				if (m_nodes.at(parentHandle).parent != NOT_FOUND)
				{
					Node& grandma = m_nodes.at(parent.parent);

					const UInt32 auntIdx = 1 - (grandma.children[1] == parentHandle);
					const UInt32 auntHandle = grandma.children[auntIdx];
					Node& aunt = m_nodes.at(auntHandle);

					const Float currentSA = nextBounds.surfaceArea();

					const Bounds rotatedBounds = aunt.bounds.merged(sibling.bounds);
					const Float rotatedSA = rotatedBounds.surfaceArea();

					// rotate
					if (rotatedSA < currentSA)
					{
						grandma.children[auntIdx] = handle;
						node.parent = parent.parent;

						parent.children[1 - sibIdx] = auntHandle;
						aunt.parent = parentHandle;

						nextBounds = rotatedBounds;
					}
				}
			}
			handle = parentHandle;
		}
	}
#pragma endregion // Private
}