#ifndef FRAME_H
#define FRAME_H

#include "Pose.h"

namespace Positional
{
	struct VelocityPose
	{
		Vec3 linear;
		Vec3 angular;

		inline VelocityPose &operator=(const VelocityPose &rhs)
		{
			linear = rhs.linear;
			angular = rhs.angular;
			return *this;
		}
	};
}
#endif // FRAME_H