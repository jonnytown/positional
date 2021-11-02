/*
 * GJK EPA Narrowphase implementation
 */
#ifndef GJK_EPA_NARROWPHASE_H
#define GJK_EPA_NARROWPHASE_H

#include "INarrowphase.h"

using namespace std;

namespace Positional::Collision
{
	class GJKEPANarrowphase : public INarrowphase
	{
	public:
		virtual bool compute(const Collider &a, const Collider &b, ContactPoint &outContact) const override;
		virtual PenetrationFunction getComputeFunction(const Collider &a, const Collider &b) const override;
	};
}
#endif // GJK_EPA_NARROWPHASE_H