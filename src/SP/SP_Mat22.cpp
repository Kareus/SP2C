#include "SP_Mat22.h"

namespace SP2
{
	SP_Mat22::SP_Mat22(double m00, double m01, double m10, double m11) : m00(m00), m01(m01), m10(m10), m11(m11)
	{

	}

	SP_Mat22::SP_Mat22(Vec2 x, Vec2 y)
	{
		xCol = x;
		yCol = y;
	}

	SP_Mat22::SP_Mat22(double radian)
	{
		double c = std::cos(radian);
		double s = std::sin(radian);

		m00 = c, m01 = -s;
		m10 = s, m11 = c;
	}

	Vec2 SP_Mat22::operator*(Vec2 v) const
	{
		return Vec2(m00 * v.x + m01 * v.y, m10 * v.x + m11 * v.y);
	}

	SP_Mat22 SP_Mat22::Transpose() const
	{
		return SP_Mat22(m00, m10, m01, m11);
	}
}