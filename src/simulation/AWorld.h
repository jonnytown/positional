#ifndef AWORLD_H
#define AWORLD_H

#include "math/Math.h"
#include "RigidBody.h"
#include <unordered_set>
#include "Store.h"

using namespace std;

namespace Positional
{
	class AWorld
	{
	private:
		Store<RigidBody> m_rigidBodies;
	public:
		const Store<RigidBody> &rigidBodies() { return m_rigidBodies; }
	};
}
#endif // AWORLD_H