#include "Body.h"
#include "mass/Computer.h"
#include <stdexcept>

namespace Positional
{
	const Store<Body>::Ref Body::null = Store<Body>::Ref();
#pragma optimize("", off)
	void Body::updateMass()
	{
		Mass::Computer computer;
		for (UInt32 i = 0, count = m_colliders.size(); i < count; ++i)
		{
			Mass::Computer it;
			m_colliders[i].get().computeMass(it);
			computer.add(it);
		}

		Vec3 com, inertia;
		Quat rot;
		Float mass;
		if (computer.diagonalize(inertia, rot, com, mass))
		{
			massPose.position = com;
			massPose.rotation = rot;
			invInertia = 1 / inertia;
			invMass = 1 / mass;
		}
		else
		{
			throw std::runtime_error("could not diagonalize inertia tensor");
		}
	}
#pragma optimize("", on)

	Float Body::getInverseMass(const Vec3 &normal, const optional<Vec3> &pos)
	{
		Vec3 n;
		Float w = 0;
		if (pos.has_value())
		{
			n = (pos.value() - pose.transform(massPose.position)).cross(normal.normalized());
			w = invMass;
		}
		else
		{
			n = normal.normalized();
		}

		n = massPose.inverseRotate(pose.inverseRotate(n));
		w += n.x * n.x * invInertia.x +
			 n.y * n.y * invInertia.y +
			 n.z * n.z * invInertia.z;

		return w;
	}
#pragma optimize("", off)
	void Body::applyRotation(const Vec3 &rot, const Float &scale)
	{
		// clamp max rotations per substep
		const Float maxPhi = 0.5;
		const Float phi = rot.length();
		Float qh = scale;
		if (phi * scale > maxPhi)
		{
			qh = maxPhi / phi;
		}

		const Vec3 worldCOM = pose.transform(massPose.position);

		// delta rotation
		Quat dq = Quat(rot.x * qh, rot.y * qh, rot.z * qh, 0);
		dq = dq * pose.rotation;
		pose.rotation = Quat(
			pose.rotation.x + 0.5 * dq.x,
			pose.rotation.y + 0.5 * dq.y,
			pose.rotation.z + 0.5 * dq.z,
			pose.rotation.w + 0.5 * dq.w
		).normalize();

		// maintain center of mass position in world space
		pose.position = pose.position + (worldCOM - pose.transform(massPose.position));
	}

	void Body::applyCorrection(const Vec3 &correction, const optional<Vec3> &pos, const bool &velLevel)
	{
		Vec3 dq;
		if (pos.has_value())
		{
			if (velLevel)
			{
				velocity.linear = velocity.linear + correction * invMass;
			}
			else
			{
				pose.position = pose.position + correction * invMass;
			}

			dq = (pos.value() - pose.transform(massPose.position)).cross(correction);
		}
		else
		{
			dq = correction;
		}

		//const Vec3 I = pose.rotate(massPose.rotate(invInertia));
		//dq.x *= I.x;
		//dq.y *= I.y;
		//dq.z *= I.z;

		dq = massPose.inverseRotate(pose.inverseRotate(dq));
		dq.x *= invInertia.x;
		dq.y *= invInertia.y;
		dq.z *= invInertia.z;
		dq = pose.rotate(massPose.rotate(dq));

		if (velLevel)
		{
			velocity.angular = velocity.angular + dq;
		}
		else
		{
			applyRotation(dq);
		}
	}
#pragma optimize("", on)
}