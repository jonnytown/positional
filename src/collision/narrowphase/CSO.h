/*
 * a simplex shape with CSO functions
 */
#ifndef CSO_H
#define CSO_H

#include "collision/collider/Collider.h"

namespace Positional::Collision
{
	struct CSO
	{
		static const UInt8 segs[6][2];
		static const UInt8 tris[4][3];
		static const UInt8 triSegs[4][3];

		Vec3 simplex[4];
		Vec3 simplexA[4];
		Vec3 simplexB[4];
		UInt8 count;

		CSO()
		{
			count = 0;
		}

		CSO(const Vec3 &_p, const Vec3 &_a, const Vec3 &_b)
		{
			simplex[0] = _p;
			simplexA[0] = _a;
			simplexB[0] = _b;
			count = 1;
		}

		inline void add(const Vec3 &_p, const Vec3 &_a, const Vec3 &_b)
		{
			assert(count < 4);
			simplex[count] = _p;
			simplexA[count] = _a;
			simplexB[count] = _b;
			count++;
		}

		inline Vec3 &operator[](UInt8 i)
		{
			assert(i < 4);
			return simplex[i];
		}

		static inline void support(const Collider &a, const Collider &b, const Vec3 &axis, Vec3 &outSupport, Vec3 &outSupportA, Vec3 &outSupportB)
		{
			const Vec3 axisA = a.vectorToLocal(axis);
			const Vec3 axisB = b.vectorToLocal(-axis);

			outSupportA = a.localSupport(axisA);
			outSupportB = b.localSupport(axisB);

			outSupport = a.pointToWorld(outSupportA) - b.pointToWorld(outSupportB);
		}

		Vec3 nearest(UInt8 &outSimplexDimension, UInt8 &outSimplexIndex) const;
		void reduce(const UInt8 &simplexDimension, const UInt8 &simplexIndex);

	private:
		Vec3 nearestOnSegment(const UInt8 &segIdx, UInt8 &outDimension, UInt8 &outIndex) const;
		Vec3 nearestOnTriangle(const UInt8 &triIdx, UInt8 &outDimension, UInt8 &outIndex) const;
		Vec3 nearestOnTetrahedron(UInt8 &outDimension, UInt8 &outIndex) const;
	};
}
#endif // CSO_H