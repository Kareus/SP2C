#pragma once
#ifndef __SP_Vector__
#define __SP_Vector__

#include <tuple>

namespace SP2
{
	struct Vec2
	{
		double x;
		double y;

		Vec2(double x = 0, double y = 0) : x(x), y(y) {}

		Vec2 operator+(const Vec2& a) const
		{
			return { x + a.x, y + a.y };
		}

		Vec2& operator+=(const Vec2& a)
		{
			x += a.x;
			y += a.y;
			return *this;
		}

		Vec2 operator-(const Vec2& a) const
		{
			return { x - a.x, y - a.y };
		}

		Vec2& operator-=(const Vec2& a)
		{
			x -= a.x;
			y -= a.y;
			return *this;
		}

		Vec2 operator*(double k) const
		{
			return { x * k,y * k };
		}

		Vec2& operator*=(double k)
		{
			x *= k;
			y *= k;
			return *this;
		}

		Vec2 operator/(double k) const
		{
			return { x / k,y / k };
		}

		Vec2& operator/=(double k)
		{
			x /= k;
			y /= k;
			return *this;
		}

		double LengthSquared()
		{
			return x * x + y * y;
		}

		double Length()
		{
			return std::sqrt(LengthSquared());
		}

		void Normalize()
		{
			double L = Length();
			if (L == 0) return;
			x /= L;
			y /= L;
		}

		bool operator==(const Vec2& a) const
		{
			return x == a.x && y == a.y;
		}

		bool operator!=(const Vec2& a) const
		{
			return !(*this == a);
		}

		Vec2 operator-() const
		{
			return { -x, -y };
		}
	};

	Vec2 operator*(double k, const Vec2& v)
	{
		return { k * v.x, k * v.y };
	}

}
#endif