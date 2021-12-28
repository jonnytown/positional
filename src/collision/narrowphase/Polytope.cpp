#include "Polytope.h"

namespace Positional::Collision::Polytope
{
	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	void init<VERT_CAP, TRI_CAP>(CSO<VERT_CAP, TRI_CAP> &poly)
	{
		poly.vertCount = 4;
		poly.triCount = 4;

		// fix winding so that v[0] - v[1] - v[2] is CCW
		const Vec3 U = poly.vertices[0].p - poly.vertices[3].p;
		const Vec3 V = poly.vertices[1].p - poly.vertices[3].p;
		const Vec3 W = poly.vertices[2].p - poly.vertices[3].p;
		const Float det = U.dot(V.cross(W));
		if (det < 0.0)
		{
			// if the determinant is negative it means our face normals are facing outwards
			poly.tris[0] = 1;
			poly.tris[1] = 0;
			poly.tris[2] = 2;
			poly.tris[3] = 1;
			poly.tris[4] = 3;
			poly.tris[5] = 0;
			poly.tris[6] = 1;
			poly.tris[7] = 2;
			poly.tris[8] = 3;
			poly.tris[9] = 0;
			poly.tris[10] = 3;
			poly.tris[11] = 2;
		}
		else
		{
			poly.tris[0] = 0;
			poly.tris[1] = 1;
			poly.tris[2] = 2;
			poly.tris[3] = 0;
			poly.tris[4] = 3;
			poly.tris[5] = 1;
			poly.tris[6] = 0;
			poly.tris[7] = 2;
			poly.tris[8] = 3;
			poly.tris[9] = 1;
			poly.tris[10] = 3;
			poly.tris[11] = 2;
		}

		for (UInt32 i = 0; i < poly.triCount; i++)
		{
			UInt32 t = i * 3;
			poly.normals[i] = GeomUtil::normal(poly.vertices[poly.tris[t]].p, poly.vertices[poly.tris[t + 1]].p, poly.vertices[poly.tris[t + 2]].p);
		}
	}

	inline Vec3 nearestOnSegment(const Vec3 &a, const Vec3 &b)
	{
		const Vec3 u = b - a;
		const Vec3 w = -a;
		const Float t = w.dot(u) / u.lengthSq();
		return a + u * Math::clamp(t, 0, 1);
	}

	Vec3 nearestOnTriangle(const Vec3 &a, const Vec3 &b, const Vec3 &c)
	{
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
			return a + p;
		}

		if (s < 0)
		{
			return nearestOnSegment(a, b);
		}

		if (t < 0)
		{
			return nearestOnSegment(c, a);
		}

		return nearestOnSegment(b, c);
	}

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	Vec3 nearest<VERT_CAP, TRI_CAP>(const CSO<VERT_CAP, TRI_CAP> &poly, Float &outLenSq, UInt32 &outTriIndex)
	{
		Vec3 nearest = nearestOnTriangle(
			poly.vertices[poly.tris[0]].p,
			poly.vertices[poly.tris[1]].p,
			poly.vertices[poly.tris[2]].p);
		outLenSq = nearest.lengthSq();
		outTriIndex = 0;

		for (UInt32 i = 3, count = poly.triCount * 3; i < count; i += 3)
		{
			const Vec3 near = nearestOnTriangle(
				poly.vertices[poly.tris[i]].p,
				poly.vertices[poly.tris[i + 1]].p,
				poly.vertices[poly.tris[i + 2]].p);

			const Float dSq = near.lengthSq();
			if (dSq < outLenSq)
			{
				outLenSq = dSq;
				nearest = near;
				outTriIndex = i/3;
			}
		}

		return nearest;
	}

	inline void addEdge(UInt32 edges[], UInt32 &edgeCount, const UInt32 &a, const UInt32 &b)
	{
		// does the opposite edge exist? if so, remove that edge and return early
		for (UInt32 i = 0, count = edgeCount * 2; i < count; i += 2)
		{
			if (edges[i] == b && edges[i + 1] == a)
			{
				// fast erase
				if (i < count - 2)
				{
					UInt32 lastIndex = count - 2;
					edges[i] = edges[lastIndex];
					edges[i+1] = edges[lastIndex+1];
				}
				edgeCount--;
				return;
			}
		}

		edges[edgeCount*2] = a;
		edges[edgeCount*2+1] = b;
		edgeCount++;
	}

	inline void clearTri(UInt32 tris[], Vec3 normals[], UInt32 &triCount, UInt32 edges[], UInt32 &edgeCount, const UInt32 &triIndex)
	{
		const UInt32 t = triIndex * 3;

		addEdge(edges, edgeCount, tris[t], tris[t + 1]);
		addEdge(edges, edgeCount, tris[t + 1], tris[t + 2]);
		addEdge(edges, edgeCount, tris[t + 2], tris[t]);

		if (triIndex < triCount - 1)
		{
			// fast erase
			const UInt32 lastIndex = (triCount - 1) * 3;
			tris[t] = tris[lastIndex];
			tris[t + 1] = tris[lastIndex + 1];
			tris[t + 2] = tris[lastIndex + 2];

			normals[triIndex] = normals[triCount -1];
		}
		triCount--;
	}

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	void expand<VERT_CAP, TRI_CAP>(CSO<VERT_CAP, TRI_CAP> &poly, const Vec3 &p, const Vec3 &a, const Vec3 &b, const UInt32 &startTriIndex)
	{
		UInt32 edges[VERT_CAP*2];
		UInt32 edgeCount = 0;
		clearTri(poly.tris, poly.normals, poly.triCount, edges, edgeCount, startTriIndex);

		// clear additional tris that are facing the new point
		for (UInt32 i = poly.triCount; i > 0; --i)
		{
			const UInt32 triIdx = i - 1;
			const UInt32 idx = triIdx * 3;
			const Float dot = poly.normals[triIdx].dot(p - poly.vertices[poly.tris[idx]].p);
			if (dot > 0)
			{
				clearTri(poly.tris, poly.normals, poly.triCount, edges, edgeCount, triIdx);
			}
		}

		// patch hole
		for (UInt32 i = 0, count = edgeCount*2; i < count; i += 2)
		{
			const Vec3 n = GeomUtil::normal(p, poly.vertices[edges[i]].p, poly.vertices[edges[i + 1]].p);
			const UInt32 t = poly.triCount * 3;
			poly.tris[t] = poly.vertCount;
			// keep winding
			if (n.dot(p) > 0)
			{
				poly.tris[t + 1] = edges[i];
				poly.tris[t + 2] = edges[i + 1];
				poly.normals[poly.triCount] = n;
			}
			else
			{
				poly.tris[t + 1] = edges[i + 1];
				poly.tris[t + 2] = edges[i];
				poly.normals[poly.triCount] = -n;
			}
			poly.triCount++;
		}

		// add vertex
		poly.vertices[poly.vertCount] = {p, a, b};
		poly.vertCount++;
	}
} // namespace Positional::Collision
