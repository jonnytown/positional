/*
 * Interface for narrowphase
 */
#ifndef INARROWPHASE_H
#define INARROWPHASE_H

#include "collision/collider/Collider.h"
#include "ContactPoint.h"
#include <functional>

using namespace std;

namespace Positional::Collision
{
	typedef function<bool(const Collider &, const Collider &, ContactPoint &)> PenetrationFunction;

	class INarrowphase
	{
	public:
		virtual ~INarrowphase(){};
		virtual bool compute(const Collider &a, const Collider &b, ContactPoint &outContact) const = 0;
		virtual PenetrationFunction getComputeFunction(const Collider &a, const Collider &b) const = 0;
	};
}
#endif // INARROWPHASE_H
