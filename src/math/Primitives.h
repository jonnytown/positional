#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include "float.h"

namespace Positional
{
#ifdef SINGLE_PRECISION
	typedef float Float;
	const Float FLOAT_MAX = FLT_MAX;
#else
	typedef double Float;
	const Float FLOAT_MAX = DBL_MAX;
#endif

	typedef char Int8;
	typedef unsigned char UInt8;

	typedef short Int16;
	typedef unsigned short UInt16;

	typedef int Int32;
	typedef unsigned int UInt32;

	typedef long Int64;
	typedef unsigned long UInt64;

	typedef long long Int128;
	typedef unsigned long long UInt128;

	const UInt32 NOT_FOUND = 0xffffffffui32;
}

#endif // PRIMITIVES_H