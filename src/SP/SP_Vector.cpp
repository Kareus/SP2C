#include "SP_Vector.h"

namespace SP2
{
	Vec2::Vec2(double x, double y) : x(x), y(y)
	{

	}

	Vec2 Vec2::operator+(const Vec2& a) const
	{
		return { x + a.x, y + a.y };
	}

	Vec2& Vec2::operator+=(const Vec2& a)
	{
		x += a.x;
		y += a.y;
		return *this;
	}

	Vec2 Vec2::operator-(const Vec2& a) const
	{
		return { x - a.x, y - a.y };
	}

	Vec2& Vec2::operator-=(const Vec2& a)
	{
		x -= a.x;
		y -= a.y;
		return *this;
	}

	Vec2 Vec2::operator*(double k) const
	{
		return { x * k,y * k };
	}

	Vec2& Vec2::operator*=(double k)
	{
		x *= k;
		y *= k;
		return *this;
	}

	Vec2 Vec2::operator/(double k) const
	{
		return { x / k,y / k };
	}

	Vec2& Vec2::operator/=(double k)
	{
		x /= k;
		y /= k;
		return *this;
	}

	double Vec2::LengthSquared()
	{
		return x * x + y * y;
	}

	double Vec2::Length()
	{
		return std::sqrt(LengthSquared());
	}

	void Vec2::Normalize()
	{
		double L = Length();
		if (L == 0) return;
		x /= L;
		y /= L;
	}

	bool Vec2::operator==(const Vec2& a) const
	{
		return x == a.x && y == a.y;
	}

	bool Vec2::operator!=(const Vec2& a) const
	{
		return !(*this == a);
	}

	Vec2 Vec2::operator-() const
	{
		return { -x, -y };
	}

	Vec2 operator*(double k, const Vec2& v)
	{
		return { k * v.x, k * v.y };
	}
}