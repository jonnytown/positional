/*
 * Interface for broadphase implementations
 */
#ifndef ABROADPHASE_H
#define ABROADPHASE_H

#include "collision/collider/ACollider.h"
#include <vector>

using namespace std;

namespace Positional::Collision
{
	class ABroadphase
	{
	public:
		virtual ~ABroadphase() = 0;
		virtual UInt32 add(Collider *collider) = 0;
		virtual UInt32 addStatic(Collider *collider) = 0;
		virtual void remove(const UInt32 &handle) = 0;
		virtual void removeStatic(const UInt32 &handle) = 0;
		virtual void update() = 0;

		virtual void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<Collider*> &results) const = 0;
		virtual void generateOverlapPairs(vector<pair<Collider *, Collider *>> &results) const = 0;
	};
}
#endif // ABROADPHASE_H
