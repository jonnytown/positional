/*
 * Polytope object for the epa algorithm
 */
#ifndef POLYTOPE_H
#define POLYTOPE_H

#include "math/Math.h"
#include <vector>

using namespace std;

namespace Positional::Collision
{
	struct Polytope
	{
		vector<Vec3> vertices;
		vector<Vec3> verticesA;
		vector<Vec3> verticesB;
		vector<UInt16> tris;
		vector<Vec3> normals;

		Polytope(const Vec3 _vertices[4], const Vec3 _verticesA[4], const Vec3 _verticesB[4]);
		Vec3 nearest(Float &outLenSq, UInt16 &outTriIndex) const;
		void expand(const Vec3 &p, const Vec3 &a, const Vec3 &b, const UInt16 &startTriIndex);
	};
}
#endif // POLYTOPE_H