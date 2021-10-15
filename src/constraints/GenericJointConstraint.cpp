#include "GenericJointConstraint.h"

namespace Positional
{
	#pragma optimize("", off)
	inline void getPositions(const Constraint &constraint, const GenericJointConstraint::Data *data, Vec3 &outPosA, Vec3 &outPosB)
	{
		outPosA = data->poseA.position;

		if (constraint.bodyA.valid())
		{
			outPosA = constraint.bodyA.get().pose.transform(outPosA);
		}

		outPosB = data->poseB.position;
		if (constraint.bodyB.valid())
		{
			outPosB = constraint.bodyB.get().pose.transform(outPosB);
		}
	}

	inline void getRotations(const Constraint &constraint, const GenericJointConstraint::Data *data, Quat &outRotA, Quat &outRotB)
	{
		outRotA = data->poseA.rotation;

		if (constraint.bodyA.valid())
		{
			outRotA = constraint.bodyA.get().pose.rotation * outRotA;
		}

		outRotB = data->poseB.rotation;
		if (constraint.bodyB.valid())
		{
			outRotB = constraint.bodyB.get().pose.rotation * outRotB;
		}
	}

	inline void applyAngleLimits(Constraint &constraint, const Vec3 &normal, const Float &compliance, const Vec3 &a, const Vec3 &b, const Float &min, const Float &max, const Float &dtInvSq, const Float maxCorr = Math::Pi)
	{
		const Vec3 c = a.cross(b);

		Float phi = Math::asin(c.dot(normal));
		if (a.dot(b) < 0)
		{
			phi = Math::Pi - phi;
		}

		if (phi > Math::Pi)
		{
			phi -= 2.0 * Math::Pi;
		}

		if (phi < - Math::Pi)
		{
			phi += 2.0 * Math::Pi;
		}

		if (phi < min || phi > max)
		{
			phi = Math::clamp(phi, min, max);

			const Quat q = Quat::fromAngleAxis(phi, normal);
			Vec3 omega = (q * a).cross(b);

			phi = omega.length();
			if (phi > maxCorr)
			{
				omega = omega * maxCorr / phi;
			}

			constraint.applyCorrections(omega, compliance, dtInvSq);
		}
	}

	void GenericJointConstraint::solvePositions(Constraint &constraint, const Float &dtInvSq)
	{
		Data *d = constraint.getData<Data>();

		// fixed rotation
		if ((d->dof & (DOF::Swing | DOF::Twist)) == 0)
		{
			Quat rotA, rotB;
			getRotations(constraint, d, rotA, rotB);
			const Quat q = rotB * rotA.conjugate();
			Vec3 omega = q.w < 0.0 ? Vec3(-2.0 * q.x, -2.0 * q.y, -2.0 * q.z) : Vec3(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
			constraint.applyCorrections(omega, d->rotationCompliance, dtInvSq);
		}
		// spherical rotation
		else if ((d->dof & DOF::Swing) == DOF::Swing)
		{
			if ((d->hasLimits & DOF::Swing) == DOF::Swing)
			{
				Quat rotA, rotB;
				getRotations(constraint, d, rotA, rotB);
				const Vec3 axisA = rotA * Vec3::pos_x;
				const Vec3 axisB = rotB * Vec3::pos_x;
				const Vec3 axb = axisA.cross(axisB);
				applyAngleLimits(constraint, axb, d->rotationCompliance, axisA, axisB, d->minSwing, d->maxSwing, dtInvSq);
			}

			bool twistDOF = (d->dof & DOF::Twist) == DOF::Twist;
			if (!twistDOF || (d->hasLimits & DOF::Twist) == DOF::Twist)
			{
				Quat rotA, rotB;
				getRotations(constraint, d, rotA, rotB);

				Float minTwist = d->minTwist;
				Float maxTwist = d->maxTwist;
				if (!twistDOF)
				{
					minTwist = maxTwist = 0;
				}

				const Vec3 nA = rotA * Vec3::pos_x;
				const Vec3 nB = rotB * Vec3::pos_x;
				Vec3 n = nA + nB;
				n.normalize();

				Vec3 axisA = rotA * Vec3::pos_y;
				axisA = axisA + n * -n.dot(axisA);
				axisA.normalize();

				Vec3 axisB = rotB * Vec3::pos_y;
				axisB = axisB + n * -n.dot(axisB);
				axisB.normalize();

				// handle gimbal lock
				const Float maxCorr = nA.dot(nB) > -0.5 ? 2.0 * Math::Pi : 1.0 / dtInvSq;

				applyAngleLimits(
					constraint,
					n,
					d->rotationCompliance,
					axisA,
					axisB,
					d->minTwist,
					d->maxTwist,
					dtInvSq,
					maxCorr);
			}
		}
		// hinge rotation (twist only)
		else if ((d->dof & DOF::Twist) == DOF::Twist)
		{
			// align axes
			Quat rotA, rotB;
			getRotations(constraint, d, rotA, rotB);
			const Vec3 axisA = rotA * Vec3::pos_x;
			const Vec3 axisB = rotB * Vec3::pos_x;
			const Vec3 axb = axisA.cross(axisB);
			constraint.applyCorrections(axb, 0.0, dtInvSq);

			if ((d->hasLimits & DOF::Twist) == DOF::Twist)
			{
				getRotations(constraint, d, rotA, rotB);

				const Vec3 normal = rotA * Vec3::pos_x;
				const Vec3 yA = rotA * Vec3::pos_y;
				const Vec3 yB = rotB * Vec3::pos_y;
				applyAngleLimits(
					constraint,
					normal,
					d->rotationCompliance,
					yA,
					yB,
					d->minTwist,
					d->maxTwist,
					dtInvSq);
			}
		}

		// fixed position
		if ((d->dof & (DOF::Linear | DOF::Planar)) == 0)
		{
			Vec3 posA, posB;
			getPositions(constraint, d, posA, posB);

			Vec3 correction = posB - posA;
			constraint.applyCorrections(correction, d->positionCompliance, dtInvSq, false, posA, posB);
		}
		// circular or spherical position
		else if ((d->dof & DOF::Planar) == DOF::Planar)
		{
			Vec3 posA, posB;

			if ((d->dof & DOF::Linear) != DOF::Linear)
			{
				getPositions(constraint, d, posA, posB);

				Quat rotA = d->poseA.rotation;
				if (constraint.bodyA.valid())
				{
					rotA = constraint.bodyA.get().pose.rotation * rotA;
				}

				const Vec3 n = rotA * Vec3::pos_x;
				const Vec3 corr = (posB - posA).project(n);

				constraint.applyCorrections(corr, d->positionCompliance, dtInvSq, false, posA, posB);
			}

			const UInt8 hasPosLimits = d->hasLimits & (DOF::Linear | DOF::Planar);

			// linear limits
			if (hasPosLimits == DOF::Linear)
			{
				getPositions(constraint, d, posA, posB);

				Quat rotA = d->poseA.rotation;
				if (constraint.bodyA.valid())
				{
					rotA = constraint.bodyA.get().pose.rotation * rotA;
				}

				const Vec3 n = rotA * Vec3::pos_x;

				Vec3 corr = (posB - posA).project(n);
				const Float lenSq = corr.lengthSq();
				if (lenSq > d->linearLimit * d->linearLimit)
				{
					const Float len = Math::sqrt(lenSq);
					corr = corr * ((len - d->linearLimit) / len);
					constraint.applyCorrections(corr, d->positionCompliance, dtInvSq, false, posA, posB);
				}
			}
			// circular limits
			else if (hasPosLimits == DOF::Planar)
			{
				getPositions(constraint, d, posA, posB);

				Quat rotA = d->poseA.rotation;
				if (constraint.bodyA.valid())
				{
					rotA = constraint.bodyA.get().pose.rotation * rotA;
				}

				const Vec3 n = rotA * Vec3::pos_x;

				Vec3 corr = (posB - posA).projectOnPlane(n);
				const Float lenSq = corr.lengthSq();
				if (lenSq > d->linearLimit * d->linearLimit)
				{
					const Float len = Math::sqrt(lenSq);
					corr = corr * ((len - d->linearLimit) / len);
					constraint.applyCorrections(corr, d->positionCompliance, dtInvSq, false, posA, posB);
				}
			}
			// spherical limits
			if (hasPosLimits != 0)
			{
				Vec3 corr = posB - posA;
				const Float lenSq = corr.lengthSq();
				if (lenSq > d->linearLimit * d->linearLimit)
				{
					const Float len = Math::sqrt(lenSq);
					corr = corr * ((len - d->linearLimit) / len);
					constraint.applyCorrections(corr, d->positionCompliance, dtInvSq, false, posA, posB);
				}
			}
		}
		// linear position
		else if ((d->dof & DOF::Linear) == DOF::Linear)
		{
			Vec3 posA, posB;
			getPositions(constraint, d, posA, posB);

			Quat rotA = d->poseA.rotation;
			if (constraint.bodyA.valid())
			{
				rotA = constraint.bodyA.get().pose.rotation * rotA;
			}

			const Vec3 n = rotA * Vec3::pos_x;
			Vec3 corr = (posB - posA).projectOnPlane(n);

			constraint.applyCorrections(corr, d->positionCompliance, dtInvSq, false, posA, posB);

			if ((d->hasLimits & DOF::Linear) == DOF::Linear)
			{
				getPositions(constraint, d, posA, posB);
				corr = (posB - posA).project(n);
				const Float lenSq = corr.lengthSq();
				if (lenSq > d->linearLimit * d->linearLimit)
				{
					corr = corr * (d->linearLimit/Math::sqrt(lenSq));
					constraint.applyCorrections(corr, d->positionCompliance, dtInvSq, false, posA, posB);
				}
			}
		}
	}

	void GenericJointConstraint::solveVelocities(Constraint &constraint, const Float &dt, const Float &dtInvSq)
	{
		Data *d = constraint.getData<Data>();

		if (d->rotationDamping > 0)
		{
			Vec3 omega(0);
			if (constraint.bodyB.valid())
			{
				omega = constraint.bodyB.get().velocity.angular;
			}

			if (constraint.bodyA.valid())
			{
				omega = omega - constraint.bodyA.get().velocity.angular;
			}

			omega = omega * Math::min(d->rotationDamping * dt, 1.0);
			constraint.applyCorrections(omega, 0, dtInvSq, true);
		}

		if (d->positionDamping > 0)
		{
			Vec3 posA = 0, posB = 0, vel = 0;

			if (constraint.bodyB.valid())
			{
				const Body &body = constraint.bodyB.get();
				posB = body.pose.transform(d->poseB.position);
				vel = body.getVelocityAt(posA);
			}

			if (constraint.bodyA.valid())
			{
				const Body &body = constraint.bodyA.get();
				posA = body.pose.transform(d->poseA.position);
				vel = vel - body.getVelocityAt(posA);
			}

			vel = vel * Math::min(d->positionDamping * dt, 1);
			constraint.applyCorrections(vel, 0, dtInvSq, true, posA, posB);
		}
	}
#pragma optimize("", on)
} // namespace Positional