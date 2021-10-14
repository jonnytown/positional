/*
 * Interface for broadphase
 */
#ifndef IBROADPHASE_H
#define IBROADPHASE_H

#include "collision/collider/Collider.h"
#include "data/Store.h"
#include <vector>

using namespace std;

namespace Positional::Collision
{
	class IBroadphase
	{
	public:
		virtual ~IBroadphase() {};
		virtual void add(const Ref<Collider> &collider) = 0;
		virtual void addStatic(const Ref<Collider> &collider) = 0;
		virtual void remove(const Ref<Collider> &collider) = 0;
		virtual void removeStatic(const Ref<Collider> &collider) = 0;
		virtual void update(const Float &dt) = 0;

		virtual void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Ref<Collider>> &results) const = 0;
		virtual void forEachOverlapPair(const function<void(pair<Ref<Collider>, Ref<Collider>>)> &callback) const = 0;
	};
}
#endif // IBROADPHASE_H
