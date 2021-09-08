#ifndef ABODY_H
#define ABODY_H

#include "math/Math.h"
#include "collision/collider/Collider.h"
#include <unordered_set>
#include <optional>
#include <functional>
#include "data/Store.h"

using namespace std;


namespace Positional
{
	class World;
	struct Body;

	struct Body final
	{
	private:
		optional<World *> m_world;

		function<Vec3(const Body &, const Vec3 &)> m_pointToWorld;
		function<Vec3(const Body &, const Vec3 &)> m_vectorToWorld;
		function<Vec3(const Body &, const Vec3 &)> m_pointToLocal;
		function<Vec3(const Body &, const Vec3 &)> m_vectorToLocal;

		Body(World *world)
		{
			m_world.emplace(world);
		}

	public:
		Vec3 position;
		Vec3 velocity;
		Quat rotation;
		Vec3 angularVelocity;

		const std::optional<World *> &world() const { return m_world; };

		Vec3 pointToWorld(const Vec3 &point) const { return m_pointToWorld(*this, point); }
		Vec3 pointToLocal(const Vec3 &point) const { return m_pointToLocal(*this, point); }
		Vec3 vectorToWorld(const Vec3 &vector) const { return m_vectorToWorld(*this, vector); }
		Vec3 vectorToLocal(const Vec3 &vector) const { return m_vectorToLocal(*this, vector); }

		void updateMassProperties();

		template <typename T>
		static Body create(World *world)
		{
			Body body = Body(world);
			body.m_pointToWorld = T::pointToWorld;
			body.m_vectorToWorld = T::vectorToWorld;
			body.m_pointToLocal = T::pointToLocal;
			body.m_vectorToLocal = T::vectorToLocal;
			return body;
		}

		static const Store<Body>::Ptr null;
	};
}
#endif // ABODY_H