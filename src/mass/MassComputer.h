#ifndef MASS_COMPUTER_H
#define MASS_COMPUTER_H

#include "math/Math.h"
#include "Volume.h"

namespace Positional::Mass
{
	class MassComputer
	{
	private:
		Mat3x3 m_inertia;
		Vec3 m_com;
		Float m_mass;

		void rotateTensor(const Quat &rotation)
		{
			// this could be optimized due to symmetry
			const Mat3x3 rot = Mat3x3::fromQuat(rotation);
			m_inertia = rot * m_inertia * rot.transposed();
		}

		void setDiagonal(const Vec3 &diagonal, const Float &mass)
		{
			m_inertia.set(
				diagonal.x, 0, 0,
				0, diagonal.y, 0,
				0, 0, diagonal.z);
			m_mass = mass;
			m_com = Vec3::zero;
		}

		Vec3 diagonalize(Quat &outRotation)
		{
			// jacobi rotation using quaternions (from an idea of Stan Melax, with fix for precision issues)

			const UInt32 MAX_ITERS = 24;

			Quat q = Quat();
			Mat3x3 d = Mat3x3();

			for (UInt32 i = 0; i < MAX_ITERS; i++)
			{
				Mat3x3 axes = Mat3x3::fromQuat(q);
				d = axes.transposed() * m_inertia * axes;

				Float d0 = Math::abs(d.m32()), d1 = Math::abs(d.m31()), d2 = Math::abs(d.m21());
				// rotation axis index, from largest off-diagonal element
				UInt8 a = d0 > d1 && d0 > d2 ? 0 : d1 > d2 ? 1 : 2;

				UInt8 a1 = (a + 1) % 3, a2 = (a1 + 1) % 3;
				if (d.get(a2, a1) == 0.0f || Math::abs(d.get(a1, a1) - d.get(a2, a2)) > 2e6 * Math::abs(2.0f * d.get(a2, a1)))
					break;

				Float w = (d.get(a1, a1) - d.get(a2, a2)) / (2 * d.get(a2, a1)); // cot(2 * phi), where phi is the rotation angle
				Float absw = Math::abs(w);

				Quat r;
				if (absw > 1000)
					r = indexedRotation(a, 1 / (4 * w), 1); // h will be very close to 1, so use small angle approx instead
				else
				{
					Float t = 1 / (absw + Math::sqrt(w * w + 1)); // absolute value of tan phi
					Float h = 1 / Math::sqrt(t * t + 1);		  // absolute value of cos phi

					assert(h != 1); // |w|<1000 guarantees this with typical IEEE754 machine eps (approx 6e-8)
					r = indexedRotation(a, Math::sqrt((1 - h) / 2) * Math::sign(w), Math::sqrt((1 + h) / 2));
				}

				q = (q * r).normalized();
			}

			outRotation = q;
			return Vec3(d.m11(), d.m22(), d.m33());
		}

		// indexed rotation around axis, with sine and cosine of half-angle
		static Quat indexedRotation(const UInt8 &axis, const Float &s, const Float &c)
		{
			Quat quat = Quat(0,0,0,c);
			quat[axis] = s;
			return quat;
		}

	public:
		MassComputer(const Mat3x3 &inertia, const Vec3 &com, const Float &mass) :
		m_inertia(inertia), m_com(com), m_mass(mass) {}

		void set(const Mat3x3 &inertia, const Vec3 &com, const Float &mass)
		{
			m_inertia = inertia;
			m_com = com;
			m_mass = mass;
		}

		void rotate(const Quat &rotation)
		{
			if (rotation.isIdentity())
			{
				return;
			}

			m_com = rotation * m_com;
			rotateTensor(rotation);
		}

		void translate(const Vec3 &translation)
		{
			//its common for this to be zero
			if (translation.x == 0 && translation.y == 0 && translation.z == 0)
			{
				return;
			}

			Mat3x3 t1, t2;
			Vec3 com = m_com;

			t1 = Mat3x3(
				0, -com.z, com.y,
				com.z, 0, -com.x,
				-com.y, com.x, 0);

			t1 = t1*t1;

			Vec3 sum = com + translation;
			if (sum.x == 0 && sum.y == 0 && sum.z == 0)
			{
				m_inertia = m_inertia + (t1 * m_mass);
			}
			else
			{
				t2 = Mat3x3(
					0, -sum.z, sum.y,
					sum.z, 0, -sum.x,
					-sum.y, sum.x, 0);

				t2 = t2*t2;

				t1 = t1 - t2 * m_mass;
				m_inertia = m_inertia + t1;
			}

			//move center of mass
			m_com = com + translation;
		}

		void transform(const Vec3 &translation, const Quat &rotation)
		{
			rotate(rotation);
			translate(translation);
		}

		void scale(const Vec3 &scale)
		{
			Mat3x3 localInertiaT(m_inertia);
			Vec3 diagonal(localInertiaT.m11(), localInertiaT.m22(), localInertiaT.m33());

			Float dot = diagonal.dot(Vec3(0.5));
			Vec3 xyz2(dot - diagonal.x, dot - diagonal.y, dot - diagonal.z); // original x^2, y^2, z^2

			Vec3 scaledxyz2(xyz2 * (scale * scale));

			Float xx = scaledxyz2.y + scaledxyz2.z,
				  yy = scaledxyz2.z + scaledxyz2.x,
				  zz = scaledxyz2.x + scaledxyz2.y;

			Float xy = localInertiaT.m21() * scale.x * scale.y,
				  xz = localInertiaT.m31() * scale.x * scale.z,
				  yz = localInertiaT.m32() * scale.y * scale.z;

			Float volumeScale = scale.x * scale.y * scale.z;

			m_inertia.set(
				xx * volumeScale, xy * volumeScale, xz * volumeScale,
				xy * volumeScale, yy * volumeScale, yz * volumeScale,
				xz * volumeScale, yz * volumeScale, zz * volumeScale);

			// scale mass
			m_mass *= volumeScale;
			m_com = m_com * scale;
		}

		void scaleDensity(const Float &densityScale)
		{
			m_inertia = m_inertia * densityScale;
			m_mass *= densityScale;
		}

		inline void center()
		{
			translate(-m_com);
		}

		void add(const MassComputer &other)
		{
			Float totalMass = m_mass + other.m_mass;
			Vec3 com = (m_com * m_mass + other.m_com * other.m_mass) / totalMass;

			m_mass = totalMass;
			m_inertia = m_inertia + other.m_inertia;
			m_com = com;
		}

		bool compute(Vec3 &outDiagTensor, Quat &outTensorRot, Vec3 &outCOM, Float &outMass)
		{
			// The inertia tensor and center of mass is relative to the actor at this point. Transform to the
			// body frame directly if CoM is specified, else use computed center of mass

			//get center of mass - has to be done BEFORE centering.
			outCOM = m_com;

			//the computed result now needs to be centered around the computed center of mass:
			center();

			// The inertia matrix is now based on the body's center of mass desc.massLocalPose.p
			outMass = m_mass;
			outDiagTensor = diagonalize(outTensorRot);

			if ((outDiagTensor.x > 0) && (outDiagTensor.y > 0) && (outDiagTensor.z > 0))
			{
				return true;
			}

			return false;
		}

		void setBox(const Vec3 &extents)
		{
			assert (extents.x > 0 && extents.y > 0 && extents.z > 0);

			// Setup inertia tensor for a cube with unit density
			Float m = boxVolume(extents);
			Float s = (1.0 / 3.0) * m;

			Float x = extents.x * extents.x;
			Float y = extents.y * extents.y;
			Float z = extents.z * extents.z;

			setDiagonal(Vec3((y + z) * s, (z + x) * s, (x + y) * s), m);
		}

		void setBox(const Vec3 &extents, const Vec3 &translation, const Quat &rotation)
		{
			setBox(extents);
			rotate(rotation);
			translate(translation);
		}

		void setSphere(const Float &radius)
		{
			Float m = sphereVolume(radius);
			// Compute moment of inertia
			Float s = m * radius * radius * (2.0 / 5.0);
			setDiagonal(Vec3(s), m);
		}

		void setSphere(const Float &radius, const Vec3 &translation, const Quat &rotation)
		{
			setSphere(radius);
			rotate(rotation);
			translate(translation);
		}

		void setCapsule(const Float &radius, const Float &length, const UInt8 &axis)
		{
			// Compute mass of capsule
			Float m = capsuleVolume(radius, length);

			Float r2 = radius * radius;
			Float l2 = length * length;
			Float lr2 = length * r2;
			Float r3x8_15 = r2 * radius * (8.0 / 15.0);

			Float t = Math::Pi * r2;
			Float i1 = t * (r3x8_15 + lr2);
			Float i2 = t * (r3x8_15 + (lr2 * (3.0 / 2.0)) + (l2 * radius * (4.0 / 3.0)) + (l2 * length * (2.0 / 3.0)));

			setDiagonal(Vec3(i2, i1, i2), m);
		}

		void setCapsule(const Float &radius, const Float &length, const UInt8 &dir, const Vec3 &translation, const Quat &rotation)
		{
			setCapsule(radius, length, dir);
			rotate(rotation);
			translate(translation);
		}
	};
}

#endif // MASS_COMPUTER_H