#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include "Primitives.h"
#include <cmath>

namespace Positional
{
	static class Math final
	{
	private:
		Math() = delete;
	public:
		static const Float Epsilon;
		static const Float Pi;
		static const Float FloatMax;
		static const Float NaN;

		static inline bool approx(const Float &a, const Float &b)
		{
			return Math::abs(a - b) < Epsilon;
		}

		static inline bool approx(const Float &a, const Float &b, const Float &epsilon)
		{
			return Math::abs(a - b) < epsilon;
		}

		/*
		 * Returns -1, 0, or 1
		 */
		static inline Float sign(const Float &x)
		{
			return (Float)(0 < x) - (Float)(x < 0);
		}
		
		/*
		 * Returns -1 or 1
		 */
		static inline Float signEq(const Float& x)
		{
			return (Float)(0 <= x) - (Float)(x < 0);
		}

		static inline Float clamp(const Float &x, const Float &mn, const Float &mx)
		{
			return Math::min(Math::max(x, mn), mx);
		}

#ifdef SINGLE_PRECISION
		static inline Float abs(const Float &x)
		{
			return ::fabsf(x);
		}

		static inline Float min(const Float &a, const Float &b)
		{
			return ::fminf(a, b);
		}

		static inline Float max(const Float &a, const Float &b)
		{
			return ::fmaxf(a, b);
		}

		static inline Float sqrt(const Float &x)
		{
			return ::sqrtf(x);
		}

		static inline Float sin(const Float &x)
		{
			return ::sinf(x);
		}

		static inline Float cos(const Float &x)
		{
			return ::cosf(x);
		}

		static inline Float tan(const Float &x)
		{
			return ::tanf(x);
		}

		static inline Float asin(const Float &x)
		{
			return ::asinf(x);
		}

		static inline Float acos(const Float &x)
		{
			return ::acosf(x);
		}

		static inline Float floor(const Float &x)
		{
			return ::floorf(x);
		}

		static inline Float ceil(const Float &x)
		{
			return ::ceilf(x);
		}
#else
		static inline Float abs(const Float &x)
		{
			return ::fabs(x);
		}

		static inline Float min(const Float &a, const Float &b)
		{
			return ::fmin(a, b);
		}

		static inline Float max(const Float &a, const Float &b)
		{
			return ::fmax(a, b);
		}

		static inline Float sqrt(const Float &x)
		{
			return ::sqrt(x);
		}

		static inline Float sin(const Float &x)
		{
			return ::sin(x);
		}

		static inline Float cos(const Float &x)
		{
			return ::cos(x);
		}

		static inline Float tan(const Float &x)
		{
			return ::tan(x);
		}

		static inline Float asin(const Float &x)
		{
			return ::asin(x);
		}

		static inline Float acos(const Float &x)
		{
			return ::acos(x);
		}

		static inline Float floor(const Float &x)
		{
			return ::floor(x);
		}

		static inline Float ceil(const Float &x)
		{
			return ::ceil(x);
		}
#endif
	};
}

#endif // MATH_UTIL_H