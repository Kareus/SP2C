#include <SP2C/SPC_Math.h>

namespace SP2C
{
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

	Vec2 GetSupport(Vec2 dir, Vec2* vertices, unsigned int n)
	{
		double bestProjection = -DBL_MAX;
		Vec2 bestVertex;

		for (unsigned int i = 0; i < n; i++)
		{
			Vec2 v = vertices[i];
			double projection = DotProduct(v, dir);

			if (projection > bestProjection)
			{
				bestVertex = v;
				bestProjection = projection;
			}
		}

		return bestVertex;
	}

	Vec2 ScaleVec(Vec2 origin, Vec2 pivot, double k)
	{
		Vec2 d = origin - pivot;
		return pivot + d * k;
	}

	Vec2 RotateVec(Vec2 origin, Vec2 pivot, double deg)
	{
		double rad = deg * Const::RAD;
		double sin = std::sin(rad), cos = std::cos(rad);

		SPC_Mat33 T_inv(1, 0, -pivot.x, 0, 1, -pivot.y, 0, 0, 1);
		SPC_Mat33 R(cos, -sin, 0, sin, cos, 0, 0, 0, 1);
		SPC_Mat33 T(1, 0, pivot.x, 0, 1, pivot.y, 0, 0, 1);

		SPC_Mat33 M = T * R * T_inv;
		return M * origin;
	}

	double ccwVal(Vec2 a, Vec2 b, Vec2 c)
	{
		return a.x * b.y + b.x * c.y + c.x * a.y - (a.y * b.x + b.y * c.x + c.y * a.x);
	}

	int ccw(Vec2 a, Vec2 b, Vec2 c)
	{
		double val = ccwVal(a, b, c);
		if (val > 0) return 1;
		else if (val < 0) return -1;
		return 0;
	}

	bool InsideTriangle(Vec2 t1, Vec2 t2, Vec2 t3, Vec2 p)
	{
		Vec2 a = t2 - t1, b = t3 - t2, c = t1 - t3;
		return (CrossProduct(a, p - t1) > 0 && CrossProduct(b, p - t2) > 0 && CrossProduct(c, p - t3) > 0);
	}
}