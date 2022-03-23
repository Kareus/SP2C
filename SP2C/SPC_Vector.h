#pragma once
#ifndef __SP2C_VECTOR__
#define __SP2C_VECTOR__

namespace SP2C
{
	struct Vec2
	{
		double x;
		double y;

		Vec2(double x = 0, double y = 0) : x(x), y(y) {}

		Vec2 operator+(Vec2 a) const
		{
			return { x + a.x, y + a.y };
		}

		Vec2& operator+=(Vec2 a)
		{
			x += a.x;
			y += a.y;
			return *this;
		}

		Vec2 operator-(Vec2 a) const
		{
			return { x - a.x, y - a.y };
		}

		Vec2& operator-=(Vec2 a)
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

		bool operator==(Vec2 a) const
		{
			return x == a.x && y == a.y;
		}

		bool operator!=(Vec2 a) const
		{
			return !(*this == a);
		}

		bool operator<(Vec2 a) const
		{
			if (x == a.x) return y < a.y;
			return x < a.x;
		}

		bool operator<=(Vec2 a) const
		{
			if (x == a.x) return y <= a.y;
			return x < a.x;
		}

		bool operator>(Vec2 a) const
		{
			if (x == a.x) return y > a.y;
			return x > a.x;
		}

		bool operator>=(Vec2 a) const
		{
			if (x == a.x) return y >= a.y;
			return x >= a.x;
		}

		Vec2 operator-() const
		{
			return { -x, -y };
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

	};

	Vec2 operator*(double k, Vec2 v)
	{
		return { k * v.x, k * v.y };
	}

	const Vec2 VEC_ZERO(0, 0);
}
#endif