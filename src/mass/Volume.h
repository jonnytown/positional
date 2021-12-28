#ifndef VOLUME_H
#define VOLUME_H

#include "math/Math.h"

namespace Positional::Volume
{
	inline static Float box(const Vec3 &extents)
	{
		return 8 * extents.x * extents.y * extents.z;
	}

	inline static Float sphere(const Float &radius)
	{
		return (4.0 / 3.0) * Math::Pi * radius * radius * radius;
	}

	inline static Float capsule(const Float &radius, const Float &length)
	{
		return sphere(radius) + Math::Pi * radius * radius * length;
	}

	inline static Float cylinder(const Float &radius, const Float &length)
	{
		return Math::Pi * radius * radius * length;
	}
}

#endif // VOLUME_H