/*
 * Interface for narrow implementations
 */
#ifndef ANARROWPHASE_H
#define ANARROWPHASE_H

#include "RaycastResult.h"
#include "CollisionResult.h"

using namespace std;

namespace Positional::Collision
{
	class ANarrowphase
	{
	public:
		virtual void raycast(const Ray &ray, const Float &maxDistance, const UInt32 &mask, vector<RaycastResult *> &results) const = 0;
		virtual void process(const vector<pair<Collider *, Collider *>> &pairs, vector<CollisionResult *> &results) const = 0;
	};
}
#endif // ANARROWPHASE_H
