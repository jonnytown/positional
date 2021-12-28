#include "Simplex.h"

namespace Positional::Collision::Simplex
{
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearestOnSegment(const CSO<VERT_CAP, TRI_CAP> &cso, const UInt8 &segIdx, UInt8 &outDimension, UInt8 &outIndex)
	{
		const UInt8 ptIdx0 = segs[segIdx][0];
		const UInt8 ptIdx1 = segs[segIdx][1];
		const Vec3 &a = cso.vertices[ptIdx0].p;
		const Vec3 &b = cso.vertices[ptIdx1].p;

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

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearestOnTriangle(const CSO<VERT_CAP, TRI_CAP> &cso, const UInt8 &triIdx, UInt8 &outDimension, UInt8 &outIndex)
	{
		const Vec3 &a = cso.vertices[Simplex::tris[triIdx][0]].p;
		const Vec3 &b = cso.vertices[Simplex::tris[triIdx][1]].p;
		const Vec3 &c = cso.vertices[Simplex::tris[triIdx][2]].p;

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

		if (s >= 0 && t >= 0 && Math::sqrt(s) + Math::sqrt(t) <= 1)
		{
			outDimension = 2;
			outIndex = triIdx;
			return a + p;
		}

		if (s < 0)
		{
			return nearestOnSegment<VERT_CAP, TRI_CAP>(cso, triSegs[triIdx][0], outDimension, outIndex);
		}

		if (t < 0)
		{
			return nearestOnSegment<VERT_CAP, TRI_CAP>(cso, triSegs[triIdx][1], outDimension, outIndex);
		}

		return nearestOnSegment<VERT_CAP, TRI_CAP>(cso, triSegs[triIdx][2], outDimension, outIndex);
	}

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearestOnTetrahedron(const CSO<VERT_CAP, TRI_CAP> &cso, UInt8 &outDimension, UInt8 &outIndex)
	{
		const Vec3 &a = cso.vertices[0].p;
		const Vec3 &b = cso.vertices[1].p;
		const Vec3 &c = cso.vertices[2].p;
		const Vec3 &d = cso.vertices[3].p;

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
			return nearestOnTriangle<VERT_CAP, TRI_CAP>(cso, 0, outDimension, outIndex);
		}

		if (t < 0)
		{
			return nearestOnTriangle<VERT_CAP, TRI_CAP>(cso, 1, outDimension, outIndex);
		}

		if (u < 0)
		{
			return nearestOnTriangle<VERT_CAP, TRI_CAP>(cso, 3, outDimension, outIndex);
		}

		return Simplex::nearestOnTriangle<VERT_CAP, TRI_CAP>(cso, 2, outDimension, outIndex);
	}

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearest<VERT_CAP, TRI_CAP>(const CSO<VERT_CAP, TRI_CAP> &cso, UInt8 &outSimplexDimension, UInt8 &outSimplexIndex)
	{
		switch (cso.vertCount)
		{
		case 1: // point
			return cso.vertices[0].p;
		case 2: // line segment
			return nearestOnSegment<VERT_CAP, TRI_CAP>(cso, 0, outSimplexDimension, outSimplexIndex);
		case 3: // triangle
			return nearestOnTriangle<VERT_CAP, TRI_CAP>(cso, 0, outSimplexDimension, outSimplexIndex);
		case 4: // tetrahedron
			return nearestOnTetrahedron<VERT_CAP, TRI_CAP>(cso, outSimplexDimension, outSimplexIndex);
		default:
			throw;
		}
	}

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	void reduce<VERT_CAP, TRI_CAP>(CSO<VERT_CAP, TRI_CAP> &cso, const UInt8 &simplexDimension, const UInt8 &simplexIndex)
	{
		if (cso.vertCount - 1 > simplexDimension)
		{
			switch (simplexDimension)
			{
			case 0:
			{
				cso.vertices[0] = cso.vertices[simplexIndex];
				cso.vertCount = 1;
				break;
			}
			case 1:
			{
				const UInt8 idx0 = segs[simplexIndex][0];
				const UInt8 idx1 = segs[simplexIndex][1];
				cso.vertices[0] = cso.vertices[idx0];
				cso.vertices[1] = cso.vertices[idx1];
				cso.vertCount = 2;
				break;
			}
			case 2:
			{
				const UInt8 idx0 = tris[simplexIndex][0];
				const UInt8 idx1 = tris[simplexIndex][1];
				const UInt8 idx2 = tris[simplexIndex][2];
				cso.vertices[0] = cso.vertices[idx0];
				cso.vertices[1] = cso.vertices[idx1];
				cso.vertices[2] = cso.vertices[idx2];
				cso.vertCount = 3;
				break;
			}
			default:
				break;
			}
		}
	}
} // namespace Positional::Collision
