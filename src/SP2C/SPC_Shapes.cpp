#include "SPC_Shapes.h"

namespace SP2C
{
	///// SPC_AABB

	SPC_AABB::SPC_AABB()
	{
		type = ShapeType::AABB;
		min = max = Vec2(0, 0);
	}

	SPC_AABB::SPC_AABB(Vec2 min, Vec2 max) : min(min), max(max)
	{
		type = ShapeType::AABB;
	}

	SPC_Shape* SPC_AABB::Clone() const
	{
		SPC_AABB* aabb = new SPC_AABB(min, max);
		return aabb;
	}

	Vec2 SPC_AABB::GetCenter() const
	{
		return 0.5 * (min + max);
	}

	Vec2 SPC_AABB::GetExtent() const
	{
		return 0.5 * (max - min);
	}

	void SPC_AABB::GetVertices(Vec2* vertices)
	{
		vertices[0] = min;
		vertices[1] = Vec2(max.x, min.y);
		vertices[2] = max;
		vertices[3] = Vec2(min.x, max.y);
	}

	void SPC_AABB::SetBox(double w, double h)
	{
		min = Vec2(-w / 2, -h / 2);
		max = Vec2(w / 2, h / 2);
	}

	void SPC_AABB::Translate(double x, double y)
	{
		min.x += x, min.y += y;
		max.x += x, max.y += y;
	}

	void SPC_AABB::Translate(Vec2 p)
	{
		min += p;
		max += p;
	}

	void SPC_AABB::Scale(double k)
	{
		Vec2 pivot = GetCenter();
		min = ScaleVec(min, pivot, k);
		max = ScaleVec(max, pivot, k);
	}

	void SPC_AABB::Transform(SPC_Mat33 matrix)
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

	SPC_AABB SPC_AABB::ComputeAABB() const
	{
		return *this;
	}

	void SPC_AABB::Combine(SPC_AABB aabb)
	{
		min.x = std::min(min.x, aabb.min.x);
		min.y = std::min(min.y, aabb.min.y);

		max.x = std::max(max.x, aabb.max.x);
		max.y = std::max(max.y, aabb.max.y);
	}

	SPC_AABB SPC_AABB::CombineAs(SPC_AABB aabb) const
	{
		return SP2C::SPC_AABB(SP2C::Vec2(std::min(min.x, aabb.min.x), std::min(min.y, aabb.min.y)), SP2C::Vec2(std::max(max.x, aabb.max.x), std::max(max.y, aabb.max.y)));
	}

	bool SPC_AABB::Contains(Vec2 v) const
	{
		return (min.x <= v.x && v.x <= max.x) && (min.y <= v.y && v.y <= max.y);
	}

	bool SPC_AABB::Contains(SPC_AABB aabb) const
	{
		return (min.x <= aabb.min.x && aabb.max.x <= max.x) && (min.y <= aabb.min.y && aabb.max.y <= max.y);
	}

	SPC_AABB CombineAABB(SPC_AABB a, SPC_AABB b)
	{
		return a.CombineAs(b);
	}

	/////
	///// SPC_Circle

	SPC_Circle::SPC_Circle(double r, Vec2 p) : radius(r), position(p)
	{
		type = ShapeType::Circle;
	}

	SPC_Shape* SPC_Circle::Clone() const
	{
		SPC_Circle* circle = new SPC_Circle(radius, position);
		return circle;
	}

	SPC_AABB SPC_Circle::ComputeAABB() const
	{
		return SPC_AABB(Vec2(position.x - radius, position.y - radius), Vec2(position.x + radius, position.y + radius));
	}

	void SPC_Circle::Translate(double x, double y)
	{
		position.x += x;
		position.y += y;
	}

	void SPC_Circle::Translate(Vec2 p)
	{
		position += p;
	}

	void SPC_Circle::Scale(double k)
	{
		radius *= k;
	}

	void SPC_Circle::Transform(SPC_Mat33 matrix)
	{
		position.x = position.x + matrix.m[0][2];
		position.y = position.y + matrix.m[1][2];

		double rot = 1; //for rotation
		if (matrix.m[1][0] != 0) rot = std::sqrt(1 - matrix.m[1][0] * matrix.m[1][0]); //sin to cos
		radius = radius * std::max(matrix.m[0][0], matrix.m[1][1]) / rot;
	}

	/////
	///// SPC_Polygon

	SPC_Polygon::SPC_Polygon()
	{
		type = ShapeType::Polygon;
		vertexCount = 0;
	}

	SPC_Shape* SPC_Polygon::Clone() const
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

	Vec2 SPC_Polygon::GetCenter()
	{
		Vec2 center;

		for (unsigned int i = 0; i < vertexCount; i++)
			center += vertices[i];

		center /= vertexCount;
		return center;
	}

	void SPC_Polygon::Set(Vec2* v, unsigned int count, bool ordering)
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

	void SPC_Polygon::SetBox(double w, double h)
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

	void SPC_Polygon::Translate(double x, double y)
	{
		for (unsigned int i = 0; i < vertexCount; i++)
			vertices[i].x += x, vertices[i].y += y;
	}

	void SPC_Polygon::Translate(Vec2 p)
	{
		for (unsigned int i = 0; i < vertexCount; i++)
			vertices[i] += p;
	}

	void SPC_Polygon::Scale(double k)
	{
		Vec2 pivot = GetCenter();
		for (unsigned int i = 0; i < vertexCount; i++)
			vertices[i] = ScaleVec(vertices[i], pivot, k);
	}

	void SPC_Polygon::Rotate(double deg)
	{
		Vec2 pivot = GetCenter();
		for (unsigned int i = 0; i < vertexCount; i++)
		{
			vertices[i] = RotateVec(vertices[i], pivot, deg);
			normals[i] = RotateVec(normals[i], VEC_ZERO, deg);
		}
	}

	void SPC_Polygon::Transform(SPC_Mat33 matrix)
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

	SPC_AABB SPC_Polygon::ComputeAABB() const
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
	
	SP2C::SPC_AABB ComputeAABB(SPC_Shape* shape)
	{
		switch (shape->type)
		{
		case SPC_Shape::AABB:
			return reinterpret_cast<SP2C::SPC_AABB*>(shape)->ComputeAABB();

		case SPC_Shape::Circle:
			return reinterpret_cast<SP2C::SPC_Circle*>(shape)->ComputeAABB();

		case SPC_Shape::Polygon:
			return reinterpret_cast<SP2C::SPC_Polygon*>(shape)->ComputeAABB();

		default:
			return SP2C::SPC_AABB();
		}
	}
}