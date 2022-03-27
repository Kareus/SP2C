#pragma once
#ifndef __SPC_SHAPES__
#define __SPC_SHAPES__

#include "SPC_Vector.h"
#include "SPC_Math.h"
#include "SPC_Mat33.h"
#include <cassert>

namespace SP2C
{

	struct SPC_Shape
	{
		enum ShapeType
		{
			AABB = 0, Circle, Polygon, Count
		};

		ShapeType type;

		virtual SPC_Shape* Clone() const = 0;
		virtual void Translate(double x, double y) {}
		virtual void Translate(Vec2 p) {}
		virtual void Scale(double k) {}
		virtual void Rotate(double deg) {}
		virtual void Transform(SPC_Mat33 matrix) {}
	};

	Vec2 AABB_normals[4] = { Vec2(0, -1), Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0) };

	struct SPC_AABB : public SPC_Shape
	{
		Vec2 min;
		Vec2 max;

		SPC_AABB()
		{
			type = ShapeType::AABB;
			min = max = Vec2(0, 0);
		}

		SPC_AABB(Vec2 min, Vec2 max) : min(min), max(max)
		{
			type = ShapeType::AABB;
		}

		SPC_Shape* Clone() const override
		{
			SPC_AABB* aabb = new SPC_AABB(min, max);
			return aabb;
		}

		Vec2 GetCenter() const
		{
			return 0.5 * (min + max);
		}

		Vec2 GetExtent() const
		{
			return 0.5 * (max - min);
		}

		void GetVertices(Vec2* vertices)
		{
			vertices[0] = min;
			vertices[1] = Vec2(max.x, min.y);
			vertices[2] = max;
			vertices[3] = Vec2(min.x, max.y);
		}

		//set aabb vertices with size. center is (0, 0)
		void SetBox(double w, double h)
		{
			min = Vec2(-w / 2, -h / 2);
			max = Vec2(w / 2, h / 2);
		}

		void Translate(double x, double y) override
		{
			min.x += x, min.y += y;
			max.x += x, max.y += y;
		}

		void Translate(Vec2 p) override
		{
			min += p;
			max += p;
		}

		void Scale(double k) override
		{
			Vec2 pivot = GetCenter();
			min = ScaleVec(min, pivot, k);
			max = ScaleVec(max, pivot, k);
		}

		void Transform(SPC_Mat33 matrix) override
		{
			Vec2 pivot = GetCenter();
			min -= pivot;
			max -= pivot;

			Vec2 v[4];
			v[0] = matrix * min;
			v[1] = matrix * Vec2(min.x, max.y);
			v[2] = matrix * Vec2(max.x, min.y);
			v[3] = matrix * max;
			
			min = max = v[0];

			for (int i = 1; i < 4; i++)
			{
				min.x = std::min(min.x, v[i].x);
				min.y = std::min(min.y, v[i].y);
				max.x = std::max(max.x, v[i].x);
				max.y = std::max(max.y, v[i].y);
			}

			min += pivot;
			max += pivot;
		}

		SPC_AABB ComputeAABB() const
		{
			return *this;
		}
	};

	struct SPC_Circle : public SPC_Shape
	{
		double radius;
		Vec2 position;

		SPC_Circle(double r = 0, Vec2 p = Vec2(0, 0)) : radius(r), position(p)
		{
			type = ShapeType::Circle;
		}

		SPC_Shape* Clone() const override
		{
			SPC_Circle* circle = new SPC_Circle(radius, position);
			return circle;
		}

		SPC_AABB ComputeAABB() const
		{
			return SPC_AABB(Vec2(-radius, -radius), Vec2(radius, radius));
		}

		void Translate(double x, double y) override
		{
			position.x += x;
			position.y += y;
		}

		void Translate(Vec2 p) override
		{
			position += p;
		}

		void Scale(double k) override
		{
			radius *= k;
		}

		void Transform(SPC_Mat33 matrix) override
		{
			position.x = position.x + matrix.m[0][2];
			position.y = position.y + matrix.m[1][2];

			double rot = 1; //for rotation
			if (matrix.m[1][0] != 0) rot = std::sqrt(1 - matrix.m[1][0] * matrix.m[1][0]); //sin to cos
			radius = radius * std::max(matrix.m[0][0], matrix.m[1][1]) / rot;
		}
	};

	struct SPC_Polygon : public SPC_Shape
	{
		static const int MAX_POLY = 64;
		Vec2 vertices[MAX_POLY];
		Vec2 normals[MAX_POLY];
		unsigned int vertexCount;

		SPC_Polygon()
		{
			type = ShapeType::Polygon;
			vertexCount = 0;
		}

		SPC_Shape* Clone() const override
		{
			SPC_Polygon* polygon = new SPC_Polygon;
			polygon->vertexCount = vertexCount;

			for (unsigned int i = 0; i < vertexCount; i++)
			{
				polygon->vertices[i] = vertices[i];
				polygon->normals[i] = normals[i];
			}

			return polygon;
		}

		Vec2 GetCenter()
		{
			Vec2 center;

			for (unsigned int i = 0; i < vertexCount; i++)
				center += vertices[i];

			center /= vertexCount;
			return center;
		}

		//set polygon vertices from vector array. result is a convex hull. you can set vertices directly with ordering=false
		void Set(Vec2* v, unsigned int count, bool ordering = true)
		{
			assert(count > 2);
			count = std::min((int)count, MAX_POLY);

			if (ordering) //vertices need to be ordered
			{
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
					for (unsigned int i = 1; i < count; i++)
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

				for (unsigned int i = 0; i < vertexCount; i++)
					vertices[i] = v[hull[i]];
			}
			else //just put vertices in original order
			{
				vertexCount = count;
				for (unsigned int i = 0; i < vertexCount; i++)
					vertices[i] = v[i];
			}

			for (unsigned int i = 0; i < vertexCount; i++)
			{
				unsigned int i2 = i + 1 < vertexCount ? i + 1 : 0;
				Vec2 face = vertices[i2] - vertices[i];
				assert(face.LengthSquared() > 1e-8);

				normals[i] = Vec2(face.y, -face.x);
				normals[i].Normalize();
			}
		}

		//set polygon vertices with size. center is (0, 0)
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

		void Translate(double x, double y) override
		{
			for (unsigned int i = 0; i < vertexCount; i++)
				vertices[i].x += x, vertices[i].y += y;
		}

		void Translate(Vec2 p) override
		{
			for (unsigned int i = 0; i < vertexCount; i++)
				vertices[i] += p;
		}

		void Scale(double k) override
		{
			Vec2 pivot = GetCenter();
			for (unsigned int i = 0; i < vertexCount; i++)
				vertices[i] = ScaleVec(vertices[i], pivot, k);
		}

		void Rotate(double deg) override
		{
			Vec2 pivot = GetCenter();
			for (unsigned int i = 0; i < vertexCount; i++)
			{
				vertices[i] = RotateVec(vertices[i], pivot, deg);
				normals[i] = RotateVec(normals[i], VEC_ZERO, deg);
			}
		}
		 
		void Transform(SPC_Mat33 matrix) override
		{
			Vec2 pivot = GetCenter();
			for (unsigned int i = 0; i < vertexCount; i++)
			{
				vertices[i] -= pivot;
				vertices[i] = matrix * vertices[i];
				vertices[i] += pivot;

				normals[i] = Vec2(matrix.m[0][0] * normals[i].x + matrix.m[0][1] * normals[i].y, matrix.m[1][0] * normals[i].x + matrix.m[1][1] * normals[i].y);
				normals[i].Normalize();
			}
		}

		SPC_AABB ComputeAABB() const
		{
			double x1 = DBL_MAX, x2 = -DBL_MAX;
			double y1 = DBL_MAX, y2 = -DBL_MAX;

			for (unsigned int i = 0; i < vertexCount; i++)
			{
				x1 = std::min(x1, vertices[i].x);
				x2 = std::max(x2, vertices[i].x);
				y1 = std::min(y1, vertices[i].y);
				y2 = std::max(y2, vertices[i].y);
			}

			return SPC_AABB(Vec2(x1, y1), Vec2(x2, y2));
		}
	};
}
#endif