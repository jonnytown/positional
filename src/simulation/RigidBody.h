#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Body.h"

namespace Positional
{
	struct RigidBody final
	{
		static Vec3 pointToWorld(const Body &body, const Vec3 &point) { return body.position + body.rotation * point; }
		static Vec3 pointToLocal(const Body &body, const Vec3 &point) { return body.rotation.inverse() * (point - body.position); }
		static Vec3 vectorToWorld(const Body &body, const Vec3 &vector) { return body.rotation * vector; }
		static Vec3 vectorToLocal(const Body &body, const Vec3 &vector) { return body.rotation.inverse() * vector; }

	private:
		RigidBody() {}
	};
}
#endif // RIGIDBODY_H