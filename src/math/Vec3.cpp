#include "Vec3.h"

namespace Positional
{
	const Vec3 Vec3::zero = Vec3();
	const Vec3 Vec3::one = Vec3(1);
	const Vec3 Vec3::neg_x = Vec3(-1, 0, 0);
	const Vec3 Vec3::pos_x = Vec3(1, 0, 0);
	const Vec3 Vec3::neg_y = Vec3(0, -1, 0);
	const Vec3 Vec3::pos_y = Vec3(0, 1, 0);
	const Vec3 Vec3::neg_z = Vec3(0, 0, -1);
	const Vec3 Vec3::pos_z = Vec3(0, 0, 1);
}