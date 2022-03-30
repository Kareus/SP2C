#pragma once
#ifndef __SP_MATH__
#define __SP_MATH__

#include <cmath>
#include "SP_Vector.h"

namespace SP2
{
	const double PI = 3.141592653589793;

	double DistanceSquared(Vec2 a, Vec2 b);

	double DotProduct(Vec2 a, Vec2 b);

	double CrossProduct(Vec2 a, Vec2 b);

	Vec2 CrossProduct(Vec2 a, double k);

	Vec2 CrossProduct(double k, Vec2 a);

	double Clamp(double value, double min, double max);

	double Pythagorean(double a, double b);
}
#endif