#pragma once
#ifndef __SP_MATH__
#define __SP_MATH__

#include <cmath>
#include "SP_Vector.h"

namespace SP2
{
	const double PI = 3.141592653589793;

	double DistanceSquared(Vec2 a, Vec2 b)
	{
		double x = a.x - b.x, y = a.y - b.y;
		return x * x + y * y;
	}

	double DotProduct(Vec2 a, Vec2 b)
	{
		return a.x * b.x + a.y * b.y;
	}

	double CrossProduct(Vec2 a, Vec2 b)
	{
		return a.x * b.y - a.y * b.x;
	}

	Vec2 CrossProduct(Vec2 a, double k)
	{
		return { k * a.y, -k * a.x };
	}

	Vec2 CrossProduct(double k, Vec2 a)
	{
		return { -k * a.y, k * a.x };
	}

	double Clamp(double value, double min, double max)
	{
		if (value <= min) return min;
		if (value >= max) return max;
		return value;
	}

	double Pythagorean(double a, double b)
	{
		return std::sqrt(a * a + b * b);
	}
}
#endif