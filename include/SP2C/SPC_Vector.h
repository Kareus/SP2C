#pragma once
#ifndef __SP2C_VECTOR__
#define __SP2C_VECTOR__

#include <cmath>

namespace SP2C
{
	struct Vec2
	{
		double x;
		double y;

		Vec2(double x = 0, double y = 0);

		Vec2 operator+(Vec2 a) const;

		Vec2& operator+=(Vec2 a);

		Vec2 operator-(Vec2 a) const;

		Vec2& operator-=(Vec2 a);

		Vec2 operator*(double k) const;

		Vec2& operator*=(double k);

		Vec2 operator/(double k) const;

		Vec2& operator/=(double k);

		bool operator==(Vec2 a) const;

		bool operator!=(Vec2 a) const;

		Vec2 operator-() const;

		double LengthSquared();

		double Length();

		void Normalize();

		bool IsZero() const;
	};

	Vec2 operator*(double k, Vec2 v);

	const Vec2 VEC_ZERO(0, 0);
}
#endif