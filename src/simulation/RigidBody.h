#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "ABody.h"

namespace Positional
{
	class RigidBody : public ABody
	{
	public:
		Quat rotation;
		Vec3 angularVelocity;

		RigidBody(const Vec3 &_position, const Quat &_rotation, Collider *const collider)
		{
			position = _position;
			rotation = _rotation;
			addCollider(collider);
		}

		virtual Vec3 pointToWorld(const Vec3 &point) const override { return position + rotation*point; }
		virtual Vec3 pointToLocal(const Vec3 &point) const override { return rotation.inverse() * (point - position); }
		virtual Vec3 vectorToWorld(const Vec3 &vector) const override { return rotation * vector; }
		virtual Vec3 vectorToLocal(const Vec3 &vector) const override { return rotation.inverse() * vector; }

		void addCollider(Collider *const collider, const bool &updateMass = true)
		{
			internal_addCollider(collider);
			if (updateMass)
			{
				updateMassProperties();
			}
		}

		void removeCollider(Collider *const collider, const bool &updateMass = true)
		{
			internal_removeCollider(collider);
			if (updateMass)
			{
				updateMassProperties();
			}
		}

		void updateMassProperties();
	};
}
#endif // RIGIDBODY_H