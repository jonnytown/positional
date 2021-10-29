#ifndef ID_PAIR_H
#define ID_PAIR_H

#include "math/Math.h"
#include <unordered_map>
#include <memory>
#include <functional>

using namespace std;

namespace Positional
{	
	template <typename T>
	struct IdPair
	{
		struct SYM_HASH
		{
			inline size_t operator()(const IdPair &pair) const
			{
				hash<T> hash;
				return hash(pair.first) + hash(pair.second);
			}
		};

		struct SYM_EQ
		{
			inline bool operator()(const IdPair &lhs, const IdPair &rhs) const
			{
				return (lhs.first == rhs.first && lhs.second == rhs.second) || (lhs.first == rhs.second && lhs.second == rhs.first);
			}
		};

		T first;
		T second;

		IdPair(const T &_first, const T &_second) : first(_first), second(_second) {}
	};
}

#endif // ID_PAIR_H