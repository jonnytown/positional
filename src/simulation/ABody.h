#ifndef ABODY_H
#define ABODY_H

#include "math/Math.h"
#include "collision/collider/ACollider.h"
#include <unordered_set>
#include <optional>

using namespace std;

namespace Positional
{
	class ABody
	{
	friend class AWorld;
	private:
		optional<AWorld *> m_world;
		unordered_set<Collider *> m_colliders;
	protected:
		void internal_addCollider(Collider *const collider)
		{
			assert(!collider->m_body.has_value());
			m_colliders.insert(collider);
			collider->m_body.emplace(this);
		}

		void internal_removeCollider(Collider *const collider)
		{
			assert(collider->m_body.has_value() && collider->m_body.value() == this);
			m_colliders.erase(collider);
			collider->m_body.reset();
		}
	public:
		Vec3 position;
		Vec3 velocity;

		virtual ~ABody() = 0;

		const std::optional<AWorld *> &world() const { return m_world; };

		const unordered_set<Collider *> &colliders() const { return m_colliders; }
		virtual Vec3 pointToWorld(const Vec3 &point) const = 0;
		virtual Vec3 pointToLocal(const Vec3 &point) const = 0;
		virtual Vec3 vectorToWorld(const Vec3 &vector) const = 0;
		virtual Vec3 vectorToLocal(const Vec3 &vector) const = 0;
	};
}
#endif // ABODY_H