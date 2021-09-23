#ifndef BODY_H
#define BODY_H

#include "math/Math.h"
#include "collision/collider/Collider.h"
#include <unordered_set>
#include <optional>
#include "data/Store.h"
#include "Frame.h"

using namespace std;
namespace Positional
{
	class World;
	struct Body;

	struct Body final
	{
		friend class World;
	private:
		optional<World *> m_world;
		vector<Store<Collider>::Ref> m_colliders;

		void (*m_integrate)(Body &, const Float &, const Vec3 &);
		void (*m_differentiate)(Body &, const Float &);

		Body(
			World *world,
			const Vec3& position,
			const Quat& rotation,
			const bool& hasRotation,
			void (*integrate)(Body &, const Float &, const Vec3 &),
			void (*differentiate)(Body&, const Float&)
		) :
			m_world(world),
			m_integrate(integrate),
			m_differentiate(differentiate),
			frame(position, rotation, hasRotation),
			prevFrame(hasRotation),
			massPose(hasRotation),
			invMass(0),
			invInertia(0) {}
	public:
		// position
		Frame frame;
		Frame prevFrame;

		// mass
		Pose massPose;
		Float invMass;
		Vec3 invInertia;

		const std::optional<World *> &world() const { return m_world; };
		const std::vector<Store<Collider>::Ref> &colliders() const { return m_colliders; }

		inline void integrate(const Float &dt, const Vec3 &gravity)
		{
			prevFrame = frame;
			m_integrate(*this, dt, gravity);
		}
		inline void differentiate(const Float &dtInv) { m_differentiate(*this, dtInv); }
		/*
		 * return inverse mass scaler at normal and optional point in world space
		 */
		Float getInverseMass(const Vec3 &normal, const optional<Vec3> &pos = std::nullopt);

		/*
		 * applies a rotation around the center of mass
		 */
		void applyRotation(const Vec3 &rot, const Float &scale = 1.0);

		/*
		 * applies a positional or velocital correction to the body at optional pos in world space
		 */
		void applyCorrection(const Vec3 &correction, const optional<Vec3> &pos = std::nullopt, const bool &velLevel = false);

		void updateMass();

		template <typename T>
		static Body create(World *world, const Vec3 &position, const Quat &rotation)
		{
			return Body(world, position, rotation, T::hasRotation(), T::integrate, T::differentiate);
		}

		static inline Vec3 pointToWorld(const Store<Body>::Ref &body, const Vec3 &point)
		{
			if (body.valid())
			{
				return body.get().frame.transform(point);
			}
			return point;
		}

		static inline Vec3 worldCom(const Store<Body>::Ref& body)
		{
			if (body.valid())
			{
				const Body& b = body.get();
				return b.frame.transform(b.massPose.position);
			}
			return Vec3::zero;
		}

		static inline void pointsToWorld(const Store<Body>::Ref &body, const Vec3 &point, Vec3 &outPrev, Vec3 &outCurr)
		{
			if (body.valid())
			{
				const Body &b = body.get();
				outPrev = b.prevFrame.transform(point);
				outCurr = b.frame.transform(point);
				return;
			}
			outPrev = outCurr = point;
		}

		static inline Vec3 pointToLocal(const Store<Body>::Ref &body, const Vec3 &point)
		{
			if (body.valid())
			{
				return body.get().frame.inverseTransform(point);
			}
			return point;
		}

		static const Store<Body>::Ref null;
	};
}
#endif // BODY_H