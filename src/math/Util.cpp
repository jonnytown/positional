#include "Util.h"

namespace Positional
{
	const Float Math::NaN = NAN;

#ifdef SINGLE_PRECISION
	const Float Math::Pi = 3.14159265358979323846f;
	const Float Math::EPSILON = 0.0000001f;
#else
	const Float Math::Pi = 3.14159265358979323846;
	const Float Math::Epsilon = 0.000000000000001;
#endif
}