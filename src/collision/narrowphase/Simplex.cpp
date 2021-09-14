#include "Simplex.h"

namespace Positional::Collision
{
	const UInt8 Simplex::segs[6][2] = {{0, 1}, {1, 2}, {0, 2}, {0, 3}, {1, 3}, {2, 3}};
	const UInt8 Simplex::tris[4][3] = {{0, 1, 2}, {0, 1, 3}, {1, 2, 3}, {0, 2, 3}};
	const UInt8 Simplex::triSegs[4][3] = {{0, 2, 1}, {0, 3, 4}, {1, 4, 5}, {2, 3, 5}};

	Vec3 Simplex::nearest(UInt8 &outSimplexDimension, UInt8 &outSimplexIndex) const
	{
		switch (count)
		{
		case 1: // point
			return vertices[0];
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

	void Simplex::reduce(const UInt8 &simplexDimension, const UInt8 &simplexIndex)
	{
		if (count - 1 > simplexDimension)
		{
			switch (simplexDimension)
			{
			case 0:
			{
				vertices[0] = vertices[simplexIndex];
				verticesA[0] = verticesA[simplexIndex];
				verticesB[0] = verticesB[simplexIndex];
				count = 1;
				break;
			}
			case 1:
			{
				const UInt8 idx0 = segs[simplexIndex][0];
				const UInt8 idx1 = segs[simplexIndex][1];
				vertices[0] = vertices[idx0];
				verticesA[0] = verticesA[idx0];
				verticesB[0] = verticesB[idx0];
				vertices[1] = vertices[idx1];
				verticesA[1] = verticesA[idx1];
				verticesB[1] = verticesB[idx1];
				count = 2;
				break;
			}
			case 2:
			{
				const UInt8 idx0 = tris[simplexIndex][0];
				const UInt8 idx1 = tris[simplexIndex][1];
				const UInt8 idx2 = tris[simplexIndex][2];
				vertices[0] = vertices[idx0];
				verticesA[0] = verticesA[idx0];
				verticesB[0] = verticesB[idx0];
				vertices[1] = vertices[idx1];
				verticesA[1] = verticesA[idx1];
				verticesB[1] = verticesB[idx1];
				vertices[2] = vertices[idx2];
				verticesA[2] = verticesA[idx2];
				verticesB[2] = verticesB[idx2];
				count = 3;
				break;
			}
			default:
				break;
			}
		}
	}

	Vec3 Simplex::nearestOnSegment(const UInt8 &segIdx, UInt8 &outDimension, UInt8 &outIndex) const
	{
		const UInt8 ptIdx0 = segs[segIdx][0];
		const UInt8 ptIdx1 = segs[segIdx][1];
		const Vec3 &a = vertices[ptIdx0];
		const Vec3 &b = vertices[ptIdx1];

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

	Vec3 Simplex::nearestOnTriangle(const UInt8 &triIdx, UInt8 &outDimension, UInt8 &outIndex) const
	{
		const Vec3 &a = vertices[tris[triIdx][0]];
		const Vec3 &b = vertices[tris[triIdx][1]];
		const Vec3 &c = vertices[tris[triIdx][2]];

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

		if (s < 0)
		{
			return nearestOnSegment(triSegs[triIdx][0], outDimension, outIndex);
		}

		if (t < 0)
		{
			return nearestOnSegment(triSegs[triIdx][1], outDimension, outIndex);
		}

		return nearestOnSegment(triSegs[triIdx][2], outDimension, outIndex);
	}

	Vec3 Simplex::nearestOnTetrahedron(UInt8 &outDimension, UInt8 &outIndex) const
	{
		const Vec3 &a = vertices[0];
		const Vec3 &b = vertices[1];
		const Vec3 &c = vertices[2];
		const Vec3 &d = vertices[3];

		const Vec3 AB = b - a;
		const Vec3 AC = c - a;
		const Vec3 AD = d - a;
		const Vec3 normABC = AB.cross(AC);

		const Vec3 AP = -a;

		const Float vol = AD.dot(normABC);
		const Float v0 = AP.dot(normABC);

		const Vec3 normABD = AB.cross(AD);
		const Float v1 = AP.dot(normABD) * Math::sign(AC.dot(normABD));

		const Vec3 normACD = AC.cross(AD);
		const Float v2 = AP.dot(normACD) * Math::sign(AB.dot(normACD));

		// barycentric coords
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

		if (s < 0)
		{
			return nearestOnTriangle(0, outDimension, outIndex);
		}

		if (t < 0)
		{
			return nearestOnTriangle(1, outDimension, outIndex);
		}

		if (u < 0)
		{
			return nearestOnTriangle(3, outDimension, outIndex);
		}

		return nearestOnTriangle(2, outDimension, outIndex);
	}
} // namespace Positional::Collision
