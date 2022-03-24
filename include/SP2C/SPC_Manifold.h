#pragma once
#ifndef __SPC_MANIFOLD__
#define __SPC_MANIFOLD__

#include "SPC_Vector.h"
#include "SPC_Shapes.h"

namespace SP2C
{
	struct SPC_Manifold
	{
		SPC_Shape* A;
		SPC_Shape* B;
		Vec2 contact_points[2];
		Vec2 normal; //normal vector from A to B
		unsigned int contact_count;
		double penetration;
	};
}
#endif