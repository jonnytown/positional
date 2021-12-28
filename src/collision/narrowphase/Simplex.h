/*
 * Configuration Space Object for the gjk algorithm
 */
#ifndef SIMPLEX_H
#define SIMPLEX_H

#include "math/Math.h"
#include "CSO.h"

namespace Positional::Collision::Simplex
{
	const UInt8 segs[6][2] = {{0, 1}, {1, 2}, {0, 2}, {0, 3}, {1, 3}, {2, 3}};
	const UInt8 tris[4][3] = {{0, 1, 2}, {0, 1, 3}, {1, 2, 3}, {0, 2, 3}};
	const UInt8 triSegs[4][3] = {{0, 2, 1}, {0, 3, 4}, {1, 4, 5}, {2, 3, 5}};

	/*
	* add a vertex to the simplex
	*/
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	static inline void add(CSO<VERT_CAP, TRI_CAP> &simplex, const Vec3 &_p, const Vec3 &_a, const Vec3 &_b)
	{
		assert(simplex.vertCount < 4);
		simplex.vertices[simplex.vertCount] = {_p, _a, _b};
		simplex.vertCount++;
	}

	/*
	* returns nearest point to the origin and the dimension and an index of the nearest simplex
	*/
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearest(const CSO<VERT_CAP, TRI_CAP> &simplex, UInt8 &outSimplexDimension, UInt8 &outSimplexIndex);

	/*
	* reduces the cso to the simplex of dimension at index. generally, use results of nearest()
	*/
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	void reduce(CSO<VERT_CAP, TRI_CAP> &simplex, const UInt8 &simplexDimension, const UInt8 &simplexIndex);
}
#endif // SIMPLEX_H