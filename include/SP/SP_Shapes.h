#pragma once
#ifndef __SP_SHAPES__
#define __SP_SHAPES__

#include "SP_Vector.h"
#include "SP_Math.h"
#include <cassert>
#include <algorithm>

namespace SP2
{
	struct SP_Shape
	{
		enum ShapeType
		{
			Circle = 0, Polygon = 1, Count
		};

		ShapeType type;
		double orientation = 0;
		Vec2 position = Vec2(0, 0);
	};

	struct SP_Circle : public SP_Shape
	{
		double radius;

		SP_Circle(double r = 0) : radius(r)
		{
			type = ShapeType::Circle;
		}
	};

	struct SP_Polygon : public SP_Shape
	{
		static const int MAX_POLY = 64;
		Vec2 vertices[MAX_POLY];
		Vec2 normals[MAX_POLY];
		unsigned int vertexCount;

		SP_Polygon();

		Vec2 GetCenter() const;

		void SetBox(double w, double h);

		void Set(Vec2* v, unsigned int count);

		void Adjust();

		Vec2 GetSupport(Vec2 dir);

	};
}
#endif