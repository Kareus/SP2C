#pragma once
#ifndef __SP_PHYSICS__
#define __SP_PHYSICS__

#include <algorithm>
#include <functional>

#include "SP_Shapes.h"
#include "SP_Body.h"
#include "SP_Manifold.h"
#include "SP_Math.h"
#include "SP_Mat22.h"

namespace SP2
{
	bool Intersect_Circle_to_Circle(SP_Manifold* manifold);

	bool Intersect_Circle_to_Polygon(SP_Manifold* manifold);

	bool Intersect_Polygon_to_Circle(SP_Manifold* manifold);

	double FindAxisLeastPenetration(unsigned int* faceIndex, SP_Body* a, SP_Body* b);

	int Clip(Vec2 n, double c, Vec2* face);

	void FindIncidentFace(Vec2* v, SP_Body* refBody, SP_Body* incBody, unsigned int referenceIndex);

	bool Intersect_Polygon_to_Polygon(SP_Manifold* manifold);

	void CorrectPosition(SP_Manifold* manifold);

	void ResolveCollision(SP_Manifold* manifold);

	typedef bool (*CollideCallback) (SP_Manifold* manifold);

	static const CollideCallback CollideTest[SP_Shape::Count][SP_Shape::Count] =
	{
		{ Intersect_Circle_to_Circle, Intersect_Circle_to_Polygon },
		{ Intersect_Polygon_to_Circle, Intersect_Polygon_to_Polygon }
	};
}
#endif