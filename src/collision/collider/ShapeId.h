#ifndef SHAPE_ID_H
#define SHAPE_ID_H

#include "math/Primitives.h"

namespace Positional
{
	namespace ShapeId
	{
		const UInt8 Box = 1 << 0;
		const UInt8 Sphere = 1 << 1;
		const UInt8 Capsule = 1 << 2;
		const UInt8 Hull = 1 << 3;
	}
}

#endif // SHAPE_ID_H