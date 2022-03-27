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

		SP_Polygon()
		{
			type = ShapeType::Polygon;
			vertexCount = 0;
		}

		Vec2 GetCenter() const
		{
			Vec2 v;
			for (unsigned int i = 0; i < vertexCount; i++)
				v += vertices[i];

			if (vertexCount) v /= vertexCount;
			return v;
		}

		void SetBox(double w, double h)
		{
			vertexCount = 4;
			vertices[0] = { -w / 2, -h / 2 };
			vertices[1] = { w / 2, -h / 2 };
			vertices[2] = { w / 2, h / 2 };
			vertices[3] = { -w / 2, h / 2 };

			normals[0] = { 0, -1 };
			normals[1] = { 1, 0 };
			normals[2] = { 0, 1 };
			normals[3] = { -1, 0 };
		}

		void Set(Vec2* v, unsigned int count)
		{
			assert(count > 2);
			count = std::min((int)count, MAX_POLY);

			int rightMost = 0;
			double highestX = v[0].x;
			for (unsigned int i = 1; i < count; i++)
			{
				double x = v[i].x;
				if (x > highestX)
				{
					highestX = x;
					rightMost = i;
				}
				else if (x == highestX)
					if (v[i].y < v[rightMost].y)
						rightMost = i;
			}

			int hull[MAX_POLY];
			int outCount = 0;
			int indexHull = rightMost;

			while (true)
			{
				hull[outCount] = indexHull;

				int nextHullIndex = 0;
				for (int i = 1; i < count; i++)
				{
					if (nextHullIndex == indexHull)
					{
						nextHullIndex = i;
						continue;
					}

					Vec2 e1 = v[nextHullIndex] - v[hull[outCount]];
					Vec2 e2 = v[i] - v[hull[outCount]];
					double c = CrossProduct(e1, e2);
					if (c < 0)
						nextHullIndex = i;

					if (c == 0 && e2.LengthSquared() > e1.LengthSquared())
						nextHullIndex = i;
				}

				outCount++;
				indexHull = nextHullIndex;
				if (nextHullIndex == rightMost)
				{
					vertexCount = outCount;
					break;
				}
			}

			for (int i = 0; i < vertexCount; i++)
				vertices[i] = v[hull[i]];

			for (int i = 0; i < vertexCount; i++)
			{
				unsigned int i2 = i + 1 < vertexCount ? i + 1 : 0;
				Vec2 face = vertices[i2] - vertices[i];
				assert(face.LengthSquared() > 1e-8);

				normals[i] = Vec2(face.y, -face.x);
				normals[i].Normalize();
			}
		}

		void Adjust()
		{
			Vec2 t = GetCenter();
			for (unsigned int i = 0; i < vertexCount; i++)
				vertices[i] -= t;

			position += t;
		}

		Vec2 GetSupport(Vec2 dir)
		{
			double bestProjection = -DBL_MAX;
			Vec2 bestVertex;

			for (unsigned int i = 0; i < vertexCount; i++)
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

	};
}
#endif