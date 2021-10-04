#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "math/Math.h"
#include "simulation/Body.h"
#include "data/Store.h"

using namespace std;
namespace Positional
{
	struct Constraint final
	{
	private:
		void (*m_solvePositions)(Constraint &, const Float &);
		void (*m_solveVelocities)(Constraint &, const Float &, const Float &);

	public:
		Store<Body>::Ref bodyA;
		Store<Body>::Ref bodyB;
		shared_ptr<void> data;

		void solvePositions(const Float &dtInvSq)
		{
			m_solvePositions(*this, dtInvSq);
		}

		void solveVelocities(const Float &dt, const Float &dtInvSq)
		{
			m_solveVelocities(*this, dt, dtInvSq);
		}

		template<typename T>
		T *getData() const { return static_cast<T*>(data.get());}

		

		template <class ConstraintT, class DataT, class... DataArgs>
		static Constraint create()
		{
			Constraint constraint;
			constraint.data = make_shared<DataT>();
			return constraint;
		}

		template <class ConstraintT, class DataT, class... DataArgs>
		void init(const Store<Body>::Ref &first, const Store<Body>::Ref &second, DataArgs &&...dataArgs)
		{
			bodyA = first;
			bodyB = second;
			getData<DataT>()->init(dataArgs...);
			m_solvePositions = ConstraintT::solvePositions;
			m_solveVelocities = ConstraintT::solveVelocities;
		}

		optional<World *> getWorld() const
		{
			if (bodyA.valid())
			{
				return bodyA.get().world();
			}

			if (bodyB.valid())
			{
				return bodyB.get().world();
			}

			return nullopt;
		}

		bool computeCorrections(
			const Vec3 &correction,
			const Float &compliance,
			const Float &dtInvSq,
			Vec3 &outNormal,
			Float &outLambda,
			const optional<Vec3> &posA = std::nullopt,
			const optional<Vec3> &posB = std::nullopt) const;

		bool computeCorrections(
			const Vec3 &correctionNormal,
			const Float& correctionLength,
			const Float& compliance,
			const Float& dtInvSq,
			Float& outLambda,
			const optional<Vec3>& posA,
			const optional<Vec3>& posB) const;

		void applyCorrections(
			const Vec3 &correction,
			const Float &compliance,
			const Float &dtInvSq,
			const bool& velLevel = false,
			const optional<Vec3> &posA = std::nullopt,
			const optional<Vec3> &posB = std::nullopt)
		{
			Vec3 n; Float lambda;
			if (computeCorrections(correction, compliance, dtInvSq, n, lambda, posA, posB))
			{
				applyCorrections(n, lambda, velLevel, posA, posB);
			}
		}

		void applyCorrections(
			const Vec3 &normal,
			const Float &lambda,
			const bool& velLevel = false,
			const optional<Vec3> &posA = std::nullopt,
			const optional<Vec3> &posB = std::nullopt)
		{
			const Vec3 correction = normal * lambda;
			if (bodyA.valid())
			{
				bodyA.get().applyCorrection(-correction, posA, velLevel);
			}

			if (bodyB.valid())
			{
				bodyB.get().applyCorrection(correction, posB, velLevel);
			}
		}

		void applyVelocityDamping(const Float& damping, const Vec3 &posA, const Vec3 &posB, const Float &dt, const Float& dtInvSq)
		{
			Vec3 vel(0);
			if (bodyB.valid())
			{
				const Body& body = bodyB.get();
				vel = body.getVelocityAt(posB);
			}

			if (bodyA.valid())
			{
				const Body& body = bodyA.get();
				vel = vel - body.getVelocityAt(posA);
			}
			vel = vel * Math::min(damping * dt, 1);
			applyCorrections(vel, 0, dtInvSq, true, optional<Vec3>(posA), optional<Vec3>(posB));
		}

		void applyAngularVelocityDamping(const Float &damping, const Float& dt, const Float& dtInvSq)
		{
			Vec3 omega(0);
			if (bodyB.valid())
			{
				omega = bodyB.get().velocity.angular;
			}

			if (bodyA.valid())
			{
				omega = omega - bodyA.get().velocity.angular;
			}

			omega = omega * Math::min(damping * dt, 1.0);
			applyCorrections(omega, 0, dtInvSq, true);
		}
	};
}
#endif // CONSTRAINT_H