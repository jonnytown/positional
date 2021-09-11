#include "CSO.h"

namespace Positional::Collision
{
	const UInt8 CSO::segs[6][2] = {{0, 1}, {1, 2}, {0, 2}, {0, 3}, {1, 3}, {2, 3}};
	const UInt8 CSO::tris[4][3] = {{0, 1, 2}, {0, 1, 3}, {1, 2, 3}, {0, 2, 3}};
	const UInt8 CSO::triSegs[4][3] = {{0, 1, 2}, {0, 3, 4}, {1, 4, 5}, {2, 3, 5}};

	Vec3 CSO::nearest(UInt8 &outSimplexDimension, UInt8 &outSimplexIndex) const
	{
		switch (count)
		{
		case 1: // point
			return simplex[0];
		case 2: // line segment
			return nearestOnSegment(0, outSimplexDimension, outSimplexIndex);
		case 3: // triangle
			return nearestOnTriangle(0, outSimplexDimension, outSimplexIndex);
		case 4: // tetrahedron
			return nearestOnTetrahedron(outSimplexDimension, outSimplexIndex);
		default:
			throw;
		}
	}

	void CSO::reduce(const UInt8 &simplexDimension, const UInt8 &simplexIndex)
	{
		if (count - 1 > simplexDimension)
		{
			switch (simplexDimension)
			{
			case 0:
			{
				simplex[0] = simplex[simplexIndex];
				simplexA[0] = simplexA[simplexIndex];
				simplexB[0] = simplexB[simplexIndex];
				count = 1;
				break;
			}
			case 1:
			{
				const UInt8 idx0 = segs[simplexIndex][0];
				const UInt8 idx1 = segs[simplexIndex][1];
				simplex[0] = simplex[idx0];
				simplexA[0] = simplexA[idx0];
				simplexB[0] = simplexB[idx0];
				simplex[1] = simplex[idx1];
				simplexA[1] = simplexA[idx1];
				simplexB[1] = simplexB[idx1];
				count = 2;
				break;
			}
			case 2:
			{
				const UInt8 idx0 = tris[simplexIndex][0];
				const UInt8 idx1 = tris[simplexIndex][1];
				const UInt8 idx2 = tris[simplexIndex][2];
				simplex[0] = simplex[idx0];
				simplexA[0] = simplexA[idx0];
				simplexB[0] = simplexB[idx0];
				simplex[1] = simplex[idx1];
				simplexA[1] = simplexA[idx1];
				simplexB[1] = simplexB[idx1];
				simplex[2] = simplex[idx2];
				simplexA[2] = simplexA[idx2];
				simplexB[2] = simplexB[idx2];
				count = 3;
				break;
			}
			default:
				break;
			}
		}
	}

	Vec3 CSO::nearestOnSegment(const UInt8 &segIdx, UInt8 &outDimension, UInt8 &outIndex) const
	{
		const UInt8 ptIdx0 = segs[segIdx][0];
		const UInt8 ptIdx1 = segs[segIdx][1];
		const Vec3 &a = simplex[ptIdx0];
		const Vec3 &b = simplex[ptIdx1];

		const Vec3 u = b - a;
		const Vec3 w = -a;
		const Float t = w.dot(u) / u.lengthSq();

		if (t < 0)
		{
			outDimension = 0;
			outIndex = ptIdx0;
			return a;
		}

		if (t > 1)
		{
			outDimension = 0;
			outIndex = ptIdx1;
			return b;
		}

		outDimension = 1;
		outIndex = segIdx;
		return a + u * t;
	}

	Vec3 CSO::nearestOnTriangle(const UInt8 &triIdx, UInt8 &outDimension, UInt8 &outIndex) const
	{
		const Vec3 &a = simplex[tris[triIdx][0]];
		const Vec3 &b = simplex[tris[triIdx][1]];
		const Vec3 &c = simplex[tris[triIdx][2]];

		const Vec3 u = b - a;
		const Vec3 v = c - a;
		const Vec3 n = u.cross(v);

		// nn is area of the triangle * 2
		const Float nn = n.lengthSq();

		// project on plane
		const Vec3 w = -a;
		const Vec3 p = w - n * (w.dot(n) / nn);

		// calculate signed areas squared (area of the parellelagram formed by two vectors)
		const Vec3 uxp = u.cross(p);
		const Vec3 pxv = p.cross(v);
		const Float areaU = uxp.lengthSq() * Math::sign(uxp.dot(n));
		const Float areaV = pxv.lengthSq() * Math::sign(pxv.dot(n));

		const Float s = areaU / nn;
		const Float t = areaV / nn;

		// TODO: figure out if it's better to avoid the square roots
		if (s >= 0 && t >= 0 && Math::sqrt(s) + Math::sqrt(t) <= 1)
		{
			outDimension = 2;
			outIndex = triIdx;
			return a + p;
		}

		Vec3 near[3];
		UInt8 dim[3], idx[3];
		near[0] = nearestOnSegment(triSegs[triIdx][0], dim[0], idx[0]);
		near[1] = nearestOnSegment(triSegs[triIdx][1], dim[1], idx[1]);
		near[2] = nearestOnSegment(triSegs[triIdx][2], dim[2], idx[2]);

		Float nearest = near[0].lengthSq();
		UInt8 nearestIdx = 0;
		for (UInt8 i = 1; i < 3; ++i)
		{
			const Float dSq = near[i].lengthSq();
			if (dSq < nearest)
			{
				nearest = dSq;
				nearestIdx = i;
			}
		}

		outDimension = dim[nearestIdx];
		outIndex = idx[nearestIdx];
		return near[nearestIdx];
	}

	Vec3 CSO::nearestOnTetrahedron(UInt8 &outDimension, UInt8 &outIndex) const
	{
		const Vec3 &a = simplex[0];
		const Vec3 &b = simplex[1];
		const Vec3 &c = simplex[2];
		const Vec3 &d = simplex[3];

		const Vec3 AB = b - a;
		const Vec3 AC = c - a;
		const Vec3 AD = d - a;
		const Vec3 normABC = AB.cross(AC);

		const Vec3 AP = -a;

		const Float vol = AD.dot(normABC);
		const Float v0 = AP.dot(normABC);

		const Vec3 normADB = AB.cross(AD);
		const Float v1 = AP.dot(normADB) * Math::sign(AC.dot(normADB));

		const Vec3 normBDC = AC.cross(AD);
		const Float v2 = AP.dot(normBDC) * Math::sign(AB.dot(normBDC));

		// barycentric coords * 6
		const Float s = v0 / vol;
		const Float t = v1 / Math::abs(vol);
		const Float u = v2 / Math::abs(vol);

		if (s >= 0 && t >= 0 && u >= 0 && s + t + u <= 1)
		{
			// origin is inside tetrahedron
			outDimension = 3;
			outIndex = 0;
			return Vec3::zero;
		}

		Vec3 near[4];
		UInt8 dim[4], idx[4];
		near[0] = nearestOnTriangle(0, dim[0], idx[0]);
		near[1] = nearestOnTriangle(1, dim[1], idx[1]);
		near[2] = nearestOnTriangle(2, dim[2], idx[2]);
		near[3] = nearestOnTriangle(3, dim[3], idx[3]);

		Float nearest = near[0].lengthSq();
		UInt8 nearestIdx = 0;
		for (UInt8 i = 1; i < 4; ++i)
		{
			Float dSq = near[i].lengthSq();
			if (dSq < nearest)
			{
				nearest = dSq;
				nearestIdx = i;
			}
		}

		outDimension = dim[nearestIdx];
		outIndex = idx[nearestIdx];
		return near[nearestIdx];
	}
} // namespace Positional::Collision
