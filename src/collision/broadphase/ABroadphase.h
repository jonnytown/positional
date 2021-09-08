/*
 * Interface for broadphase implementations
 */
#ifndef ABROADPHASE_H
#define ABROADPHASE_H

#include "collision/collider/Collider.h"
#include "data/Store.h"
#include <vector>

using namespace std;

namespace Positional::Collision
{
	class ABroadphase
	{
	public:
		virtual ~ABroadphase() {};
		virtual void add(const Store<Collider>::Ptr &collider) = 0;
		virtual void addStatic(const Store<Collider>::Ptr &collider) = 0;
		virtual void remove(const Store<Collider>::Ptr &collider) = 0;
		virtual void removeStatic(const Store<Collider>::Ptr &collider) = 0;
		virtual void update() = 0;

		virtual void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Store<Collider>::Ptr> &results) const = 0;
		virtual void generateOverlapPairs(vector<pair<Store<Collider>::Ptr, Store<Collider>::Ptr>> &results) const = 0;
	};
}
#endif // ABROADPHASE_H
