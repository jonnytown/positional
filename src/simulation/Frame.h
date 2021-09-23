#ifndef FRAME_H
#define FRAME_H

#include "Pose.h"

namespace Positional
{
	struct Frame : public Pose
	{
		Vec3 velocity;
		Vec3 angularVelocity;

		Frame(const bool &_useRotation) :
			Pose(_useRotation), velocity(Vec3::zero), angularVelocity(Vec3::zero)
		{}

		Frame(const Vec3 &_position, const Quat &_rotation, const bool &_useRotation) :
			Pose(_position, _rotation, _useRotation), velocity(Vec3::zero), angularVelocity(Vec3::zero)
		{}

		/*
		 * returns the velocity at a given point
		 */
		inline Vec3 getVelocityAt(const Vec3 &localCOM, const Vec3 &point) const
		{
			return velocity - (point - transform(localCOM)).cross(angularVelocity);
		}
	};
}
#endif // FRAME_H