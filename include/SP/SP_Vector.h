#pragma once
#ifndef __SP_Vector__
#define __SP_Vector__

#include <cmath>

namespace SP2
{
	struct Vec2
	{
		double x;
		double y;

		Vec2(double x = 0, double y = 0);

		Vec2 operator+(const Vec2& a) const;

		Vec2& operator+=(const Vec2& a);

		Vec2 operator-(const Vec2& a) const;

		Vec2& operator-=(const Vec2& a);

		Vec2 operator*(double k) const;

		Vec2& operator*=(double k);

		Vec2 operator/(double k) const;

		Vec2& operator/=(double k);

		double LengthSquared();

		double Length();

		void Normalize();

		bool operator==(const Vec2& a) const;

		bool operator!=(const Vec2& a) const;

		Vec2 operator-() const;
	};

	Vec2 operator*(double k, const Vec2& v);

}
#endif