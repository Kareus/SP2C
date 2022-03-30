#include <SP2C/SPC_Vector.h>

namespace SP2C
{
	Vec2::Vec2(double x, double y) : x(x), y(y)
	{
	}

	Vec2 Vec2::operator+(Vec2 a) const
	{
		return { x + a.x, y + a.y };
	}

	Vec2& Vec2::operator+=(Vec2 a)
	{
		x += a.x;
		y += a.y;
		return *this;
	}

	Vec2 Vec2::operator-(Vec2 a) const
	{
		return { x - a.x, y - a.y };
	}

	Vec2& Vec2::operator-=(Vec2 a)
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

	bool Vec2::operator==(Vec2 a) const
	{
		return x == a.x && y == a.y;
	}

	bool Vec2::operator!=(Vec2 a) const
	{
		return !(*this == a);
	}

	bool Vec2::operator<(Vec2 a) const
	{
		if (x == a.x) return y < a.y;
		return x < a.x;
	}

	bool Vec2::operator<=(Vec2 a) const
	{
		if (x == a.x) return y <= a.y;
		return x < a.x;
	}

	bool Vec2::operator>(Vec2 a) const
	{
		if (x == a.x) return y > a.y;
		return x > a.x;
	}

	bool Vec2::operator>=(Vec2 a) const
	{
		if (x == a.x) return y >= a.y;
		return x >= a.x;
	}

	Vec2 Vec2::operator-() const
	{
		return { -x, -y };
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

	Vec2 operator*(double k, Vec2 v)
	{
		return { k * v.x, k * v.y };
	}
}