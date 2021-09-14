#include "Polytope.h"

namespace Positional::Collision
{
	Polytope::Polytope(const Vec3 _vertices[4], const Vec3 _verticesA[4], const Vec3 _verticesB[4])
	{
		vertices.insert(vertices.end(), _vertices, _vertices + 4);
		verticesA.insert(verticesA.end(), _verticesA, _verticesA + 4);
		verticesB.insert(verticesB.end(), _verticesB, _verticesB + 4);

		// fix winding so that v[0] - v[1] - v[2] is CCW
		const Vec3 U = _vertices[0] - _vertices[3];
		const Vec3 V = _vertices[1] - _vertices[3];
		const Vec3 W = _vertices[2] - _vertices[3];
		const Float det = U.dot(V.cross(W));
		if (det < 0.0)
		{
			// if the determinant is negative it means our face normals are facing outwards
			tris = vector<UInt16>({
				1, 0, 2,
				1, 3, 0,
				1, 2, 3,
				0, 3, 2 });
		}
		else
		{
			tris = vector<UInt16>({
				0, 1, 2,
				0, 3, 1,
				0, 2, 3,
				1, 3, 2 });
		}

		for (UInt16 i = 0, count = tris.size(); i < count; i += 3)
		{
			normals.push_back(GeomUtil::normal(vertices[tris[i]], vertices[tris[i + 1]], vertices[tris[i + 2]]));
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

	Vec3 Polytope::nearest(Float &outLenSq, UInt16 &outTriIndex) const
	{
		Vec3 nearest = nearestOnTriangle(
			vertices[tris[0]],
			vertices[tris[1]],
			vertices[tris[2]]);
		outLenSq = nearest.lengthSq();
		outTriIndex = 0;

		for (UInt16 i = 3, count = tris.size(); i < count; i += 3)
		{
			const Vec3 near = nearestOnTriangle(
				vertices[tris[i]],
				vertices[tris[i+1]],
				vertices[tris[i+2]]);

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

	inline void addEdge(vector<UInt16>& edges, const UInt16& a, const UInt16& b)
	{
		// does the opposite edge exist? if so, remove that edge and return early
		for (UInt16 i = 0, count = edges.size(); i < count; i += 2)
		{
			if (edges[i] == b && edges[i + 1] == a)
			{
				const auto begin = edges.begin() + i;
				edges.erase(begin, begin + 2);
				return;
			}
		}

		edges.push_back(a);
		edges.push_back(b);
	}

	inline void clearTri(vector<UInt16> &tris, vector<Vec3> &normals, vector<UInt16> &edges, const UInt16 &triIndex)
	{
		const UInt16 index = triIndex * 3;

		addEdge(edges, tris[index], tris[index + 1]);
		addEdge(edges, tris[index + 1], tris[index + 2]);
		addEdge(edges, tris[index + 2], tris[index]);

		const auto begin = tris.begin() + index;
		tris.erase(begin, begin + 3);
		normals.erase(normals.begin() + triIndex);
	}


	void Polytope::expand(const Vec3 &p, const Vec3 &a, const Vec3 &b, const UInt16 &startTriIndex)
	{
		vector<UInt16> edges;
		clearTri(tris, normals, edges, startTriIndex);

		// clear additional tris that are facing the new point
		UInt16 triCount = tris.size() / 3;
		for (UInt16 i = triCount; i > 0; --i)
		{
			const UInt16 triIdx = i - 1;
			const UInt16 idx = triIdx * 3;
			const Float dot = normals[triIdx].dot(p - vertices[tris[idx]]);
			if (dot > 0)
			{
				clearTri(tris, normals, edges, triIdx);
			}
		}

		// add vertex
		vertices.push_back(p);
		verticesA.push_back(a);
		verticesB.push_back(b);
		UInt16 index = vertices.size() - 1;

		// patch hole
		for (UInt16 i = 0, count = edges.size(); i < count; i += 2)
		{
			const Vec3 n = GeomUtil::normal(p, vertices[edges[i]], vertices[edges[i + 1]]);
			tris.push_back(index);
			// keep winding
			if (n.dot(p) > 0)
			{
				tris.push_back(edges[i]);
				tris.push_back(edges[i + 1]);
				normals.push_back(n);
			}
			else
			{
				tris.push_back(edges[i + 1]);
				tris.push_back(edges[i]);
				normals.push_back(-n);
			}
		}
	}
	
} // namespace Positional::Collision