#pragma once
#ifndef __SPC_COLLISION__
#define __SPC_COLLISION__

#include <algorithm>
#include "SPC_Vector.h"
#include "SPC_Math.h"
#include "SPC_Shapes.h"
#include "SPC_Manifold.h"

namespace SP2C
{
	namespace Collision
	{
		int Clip(Vec2 n, double c, Vec2* face);

		double FindAxisLeastPenetration(unsigned int* faceIndex, Vec2* a, Vec2* normals, unsigned int n, Vec2* b, unsigned int m);

		void FindIncidentFace(Vec2* v, Vec2 refNormal, Vec2* incVertices, Vec2* incNormals, unsigned int n);

		bool AABB_to_AABB(SPC_AABB& a, SPC_AABB& b);

		bool AABB_to_AABB(SPC_Manifold* m);

		bool AABB_to_Circle(SPC_AABB& a, SPC_Circle& b);

		bool AABB_to_Circle(SPC_Manifold* m);

		bool Circle_to_AABB(SPC_Manifold* m);

		bool AABB_to_Polygon(SPC_AABB& a, SPC_Polygon& b);

		bool AABB_to_Polygon(SPC_Manifold* m);

		bool Polygon_to_AABB(SPC_Manifold* m);

		bool Circle_to_Circle(SPC_Circle& a, SPC_Circle& b);

		bool Circle_to_Circle(SPC_Manifold* m);

		bool Circle_to_Polygon(SPC_Circle& a, SPC_Polygon& b);

		bool Circle_to_Polygon(SPC_Manifold* m);

		bool Polygon_to_Circle(SPC_Manifold* m);

		bool Polygon_to_Polygon(SPC_Polygon a, SPC_Polygon b);

		bool Polygon_to_Polygon(SPC_Manifold* m);

		typedef bool (*SPC_CollideCallback) (SPC_Manifold* manifold);

		static const SPC_CollideCallback CollideFunc[SPC_Shape::Count][SPC_Shape::Count] =
		{
			{ AABB_to_AABB, AABB_to_Circle, AABB_to_Polygon },
			{ Circle_to_AABB, Circle_to_Circle, Circle_to_Polygon },
			{ Polygon_to_AABB, Polygon_to_Circle, Polygon_to_Polygon }
		};

		bool Collide(SPC_Manifold* m);
	}
}
#endif