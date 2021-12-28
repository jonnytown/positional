/*
 * Configuration space object for the gjk and epa algorithms
 */
#ifndef CSO_H
#define CSO_H

#include "math/Math.h"

using namespace std;

namespace Positional::Collision
{
	struct Vertex
	{
		Vec3 p, a, b;
	};

	template <UInt32 VERT_CAP, UInt32 TRI_CAP>
	struct CSO
	{
		UInt32 triCount;
		UInt32 vertCount;
		Vertex vertices[VERT_CAP];
		UInt32 tris[TRI_CAP * 3];
		Vec3 normals[TRI_CAP];

		inline Vertex &operator[](UInt8 i)
		{
			assert(i < vertCount);
			return vertices[i];
		}

		CSO()
		{
			vertCount = 0;
			triCount = 0;
		}

		CSO(const Vec3 &_p, const Vec3 &_a, const Vec3 &_b)
		{
			vertices[0].p = _p;
			vertices[0].a = _a;
			vertices[0].b = _b;
			vertCount = 1;
			triCount = 0;
		}
	};
} // namespace Positional::Collision

#endif // CSO_H