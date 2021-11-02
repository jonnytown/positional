#ifndef VOLUME_H
#define VOLUME_H

#include "math/Math.h"

namespace Positional::Mass
{
	inline static Float boxVolume(const Vec3 &extents)
	{
		return 8 * extents.x * extents.y * extents.z;
	}

	inline static Float sphereVolume(const Float &radius)
	{
		return (4.0 / 3.0) * Math::Pi * radius * radius * radius;
	}

	inline static Float capsuleVolume(const Float &radius, const Float &length)
	{
		return sphereVolume(radius) + Math::Pi * radius * radius * length;
	}

	inline static Float cylinderVolume(const Float &radius, const Float &length)
	{
		return Math::Pi * radius * radius * length;
	}
}

#endif // VOLUME_H