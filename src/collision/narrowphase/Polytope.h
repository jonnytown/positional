/*
 * Polytope object for the epa algorithm
 */
#ifndef POLYTOPE_H
#define POLYTOPE_H

#include "math/Math.h"
#include "CSO.h"

using namespace std;

namespace Positional::Collision::Polytope
{
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	void init(CSO<VERT_CAP, TRI_CAP> &poly);
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearest(const CSO<VERT_CAP, TRI_CAP> &poly, Float &outLenSq, UInt32 &outTriIndex);
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	void expand(CSO<VERT_CAP, TRI_CAP> &poly, const Vec3 &p, const Vec3 &a, const Vec3 &b, const UInt32 &startTriIndex);
}
#endif // POLYTOPE_H