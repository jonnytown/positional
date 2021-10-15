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
		Ref<Body> bodyA;
		Ref<Body> bodyB;
		shared_ptr<void> data;

		void solvePositions(const Float &dtInvSq)
		{
			m_solvePositions(*this, dtInvSq);
		}

		void solveVelocities(const Float &dt, const Float &dtInvSq)
		{
			m_solveVelocities(*this, dt, dtInvSq);
		}

		template<class T>
		T *getData() const { return static_cast<T*>(data.get());}

		template <class ConstraintT, class DataT>
		static Constraint create()
		{
			Constraint constraint;
			constraint.m_solvePositions = ConstraintT::solvePositions;
			constraint.m_solveVelocities = ConstraintT::solveVelocities;
			constraint.data = make_shared<DataT>();
			return constraint;
		}

		template <class DataT, typename... DataArgs>
		void init(const Ref<Body> &first, const Ref<Body> &second, DataArgs &&...dataArgs)
		{
			bodyA = first;
			bodyB = second;
			getData<DataT>()->init(dataArgs...);
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
	};
}
#endif // CONSTRAINT_H