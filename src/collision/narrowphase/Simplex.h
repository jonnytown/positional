/*
 * Configuration Space Object for the gjk algorithm
 */
#ifndef SIMPLEX_H
#define SIMPLEX_H

#include "math/Math.h"

namespace Positional::Collision
{
	struct Simplex
	{
		static const UInt8 segs[6][2];
		static const UInt8 tris[4][3];
		static const UInt8 triSegs[4][3];

		Vec3 vertices[4];
		Vec3 verticesA[4];
		Vec3 verticesB[4];
		UInt8 count;

		Simplex()
		{
			count = 0;
		}

		Simplex(const Vec3 &_p, const Vec3 &_a, const Vec3 &_b)
		{
			vertices[0] = _p;
			verticesA[0] = _a;
			verticesB[0] = _b;
			count = 1;
		}

		inline void add(const Vec3 &_p, const Vec3 &_a, const Vec3 &_b)
		{
			assert(count < 4);
			vertices[count] = _p;
			verticesA[count] = _a;
			verticesB[count] = _b;
			count++;
		}

		inline Vec3 &operator[](UInt8 i)
		{
			assert(i < 4);
			return vertices[i];
		}

		/*
		 * returns nearest point to the origin and the dimension and an index of the nearest simplex
		 */
		Vec3 nearest(UInt8 &outSimplexDimension, UInt8 &outSimplexIndex) const;
		/*
		 * reduces the cso to the simplex of dimension at index. generally, use results of nearest()
		 */
		void reduce(const UInt8 &simplexDimension, const UInt8 &simplexIndex);

	private:
		Vec3 nearestOnSegment(const UInt8 &segIdx, UInt8 &outDimension, UInt8 &outIndex) const;
		Vec3 nearestOnTriangle(const UInt8 &triIdx, UInt8 &outDimension, UInt8 &outIndex) const;
		Vec3 nearestOnTetrahedron(UInt8 &outDimension, UInt8 &outIndex) const;
	};
}
#endif // SIMPLEX_H