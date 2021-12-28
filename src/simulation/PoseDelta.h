#ifndef FRAME_H
#define FRAME_H

#include "Pose.h"

namespace Positional
{
	struct PoseDelta
	{
		Vec3 linear;
		Vec3 angular;

		inline PoseDelta &operator=(const PoseDelta &rhs)
		{
			linear = rhs.linear;
			angular = rhs.angular;
			return *this;
		}
	};
}
#endif // FRAME_H