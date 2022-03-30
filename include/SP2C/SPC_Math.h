#pragma once
#ifndef __SPC_MATH__
#define __SPC_MATH__

#include <cmath>
#include <algorithm>
#include "SPC_Constants.h"
#include "SPC_Vector.h"
#include "SPC_Mat33.h"

namespace SP2C
{
	double DistanceSquared(Vec2 a, Vec2 b);

	double DotProduct(Vec2 a, Vec2 b);

	double CrossProduct(Vec2 a, Vec2 b);

	Vec2 CrossProduct(Vec2 a, double k);

	Vec2 CrossProduct(double k, Vec2 a);

	double Clamp(double value, double min, double max);

	double Pythagorean(double a, double b);

	Vec2 GetSupport(Vec2 dir, Vec2* vertices, unsigned int n);

	Vec2 ScaleVec(Vec2 origin, Vec2 pivot, double k);

	Vec2 RotateVec(Vec2 origin, Vec2 pivot, double deg);

	double ccwVal(Vec2 a, Vec2 b, Vec2 c);

	int ccw(Vec2 a, Vec2 b, Vec2 c);

	bool InsideTriangle(Vec2 t1, Vec2 t2, Vec2 t3, Vec2 p);
}
#endif