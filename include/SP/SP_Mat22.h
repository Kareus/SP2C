#pragma once
#ifndef __SP_MAT22__
#define __SP_MAT22__

#include <cmath>
#include "SP_Vector.h"

namespace SP2
{
	struct SP_Mat22
	{
		union
		{
			struct
			{
				double m00, m01;
				double m10, m11;
			};

			struct
			{
				Vec2 xCol;
				Vec2 yCol;
			};
		};

		SP_Mat22(double m00, double m01, double m10, double m11);

		SP_Mat22(Vec2 x, Vec2 y);

		SP_Mat22(double radian);

		Vec2 operator*(Vec2 v) const;

		SP_Mat22 Transpose() const;
	};
}
#endif