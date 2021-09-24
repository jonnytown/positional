#ifndef POSE_H
#define POSE_H

#include "math/Math.h"

namespace Positional
{
	struct Pose
	{
	private:
		Vec3 (*m_transform)(const Pose &, const Vec3 &);
		Vec3 (*m_inverseTransform)(const Pose &, const Vec3 &);
		Vec3 (*m_rotate)(const Pose &, const Vec3 &);
		Vec3 (*m_inverseRotate)(const Pose &, const Vec3 &);

		inline static Vec3 transformNonlinear(const Pose &t, const Vec3 &point) { return t.position + t.rotation * point; }
		inline static Vec3 inverseTransformNonlinear(const Pose &t, const Vec3 &point) { return t.rotation.inverse() * (point - t.position); }
		inline static Vec3 rotateNonlinear(const Pose &t, const Vec3 &vector) { return t.rotation * vector; }
		inline static Vec3 inverseRotateNonlinear(const Pose &t, const Vec3 &vector) { return t.rotation.inverse() * vector; }

		inline static Vec3 transformLinear(const Pose &t, const Vec3 &point) { return t.position + point; }
		inline static Vec3 inverseTransformLinear(const Pose &t, const Vec3 &point) { return point - t.position; }
		inline static Vec3 rotateLinear(const Pose &t, const Vec3 &vector) { return vector; }
		inline static Vec3 inverseRotateLinear(const Pose &t, const Vec3 &vector) { return vector; }

		inline void useRotation(const bool &useRotation)
		{
			if (useRotation)
			{
				m_transform = transformNonlinear;
				m_inverseTransform = inverseTransformNonlinear;
				m_rotate = rotateNonlinear;
				m_inverseRotate = inverseRotateNonlinear;
			}
			else
			{
				m_transform = transformLinear;
				m_inverseTransform = inverseTransformLinear;
				m_rotate = rotateLinear;
				m_inverseRotate = inverseRotateLinear;
			}
		}
	
	public:
		Vec3 position;
		Quat rotation;
		bool usesRotation;

		Pose(const bool &_useRotation) :
			position(Vec3::zero),
			rotation(Quat::identity),
			usesRotation(_useRotation)
		{
			useRotation(_useRotation);
		}

		Pose(const Vec3 &_position, const Quat &_rotation, const bool &_useRotation) :
			position(_position),
			rotation(_rotation),
			usesRotation(_useRotation)
		{
			useRotation(_useRotation);
		}

		inline Pose &operator=(const Pose &rhs)
		{
			usesRotation = rhs.usesRotation;
			position = rhs.position;
			rotation = rhs.rotation;
			m_transform = rhs.m_transform;
			m_inverseTransform = rhs.m_inverseTransform;
			m_rotate = rhs.m_rotate;
			m_inverseRotate = rhs.m_inverseRotate;
			return *this;
		}

		inline Pose operator*(const Pose &rhs) const
		{
			Pose results(usesRotation | rhs.usesRotation);
			if (results.usesRotation)
			{
				results.rotation = rotation * rhs.rotation;
				results.position = rotation * rhs.position + position;
			}
			else
			{
				results.position = position + rhs.position;
			}
			return results;
		}

		inline Pose inverse() const
		{
			Pose results(usesRotation);
			results.position = -position;
			results.rotation = rotation.conjugate();
		}

		inline Vec3 transform(const Vec3 &point) const { return m_transform(*this, point); }
		inline Vec3 inverseTransform(const Vec3 &point) const { return m_inverseTransform(*this, point); }
		inline Vec3 rotate(const Vec3 &vector) const { return m_rotate(*this, vector); }
		inline Vec3 inverseRotate(const Vec3 &vector) const { return m_inverseRotate(*this, vector); }
	};
}
#endif // POSE_H