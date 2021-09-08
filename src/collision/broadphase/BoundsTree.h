/*
 * Bounds Tree (aka Bounding Volume Hierarchy)
 */
#include "math/Math.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>

#ifndef BOUNDS_TREE_H
#define BOUNDS_TREE_H

using namespace std;

namespace Positional::Collision
{
	struct HandlePair
	{
		UInt32 first;
		UInt32 second;

		HandlePair(const UInt32& _first, const UInt32& _second) : first(_first), second(_second) {}
	};

	typedef function<void(const UInt32 &)> ResultCallback;
	typedef function<void(const HandlePair &)> ResultPairCallback;

	class BoundsTree
	{
	private:
		class Node
		{
		public:
			Bounds bounds;
			UInt32 mask;
			UInt32 parent;
			UInt32 children[2];

			Node() : bounds(Bounds()), mask(0), parent(NOT_FOUND)
			{
				children[0] = children[1] = NOT_FOUND;
			}

			Node(const UInt32 &_parent)
				: bounds(Bounds()), mask(0), parent(_parent)
			{
				children[0] = children[1] = NOT_FOUND;
			}

			Node(const Bounds &_bounds, const UInt32 &_mask, const UInt32 &_parent)
			: bounds(_bounds), mask(_mask), parent(_parent)
			{
				children[0] = children[1] = NOT_FOUND;
			}

			inline bool isLeaf() const { return children[1] == NOT_FOUND; }
		};

		unordered_map<UInt32, Node> m_nodes;
		unordered_set<UInt32> m_leaves;
		UInt32 m_nextHandle;
		UInt32 m_root;

		void add(const Bounds &bounds, const UInt32 &mask, const UInt32 &handle);
		void remove(const UInt32 &handle, const bool &refitAncestors);

		UInt32 findBestSibling(const Bounds &bounds) const;
		void refit(const UInt32 &startHandle);

		inline UInt32 nextHandle() {
			return m_nextHandle++;
		}

	public:
		BoundsTree()
		{
			m_nodes = unordered_map<UInt32, Node>();
			m_leaves = unordered_set<UInt32>();
			m_nextHandle = 0;
			m_root = NOT_FOUND;
		}

		UInt32 add(const Bounds &bounds, const UInt32 &mask);
		void update(const UInt32 &handle, const Bounds &bounds, const UInt32 &mask);
		void updateMask(const UInt32 &handle, const UInt32 &mask);
		void remove(const UInt32 &handle);

		void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, const ResultCallback &resultsCallback) const;
		void intersects(const Bounds &bounds, const UInt32 &mask, const ResultCallback &resultsCallback, const bool &exclusive = false) const;
		void generateOverlapPairs(const ResultPairCallback &resultsCallback, const bool &exclusive = false) const;

		void forEachNode(const function<void(Bounds)> &callback)
		{
			for (const auto &[handle, node] : m_nodes)
			{
				callback(node.bounds);
			}
		}
	};

}

#endif // BOUNDS_TREE_H
