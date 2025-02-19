#pragma once
#ifndef __SPC_SHAPES__
#define __SPC_SHAPES__

#include <cassert>
#include <algorithm>
#include <SP2C/SPC_Vector.h>
#include <SP2C/SPC_Math.h>
#include <SP2C/SPC_Mat33.h>

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

	const Vec2 AABB_normals[4] = { Vec2(0, -1), Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0) };

	struct SPC_AABB : public SPC_Shape
	{
		Vec2 min;
		Vec2 max;

		SPC_AABB();

		SPC_AABB(Vec2 min, Vec2 max);

		SPC_AABB(const SPC_AABB& aabb);

		SPC_Shape* Clone() const override;

		Vec2 GetCenter() const;

		Vec2 GetExtent() const;

		void GetVertices(Vec2* vertices) const;

		//set aabb vertices with size. top left is (0, 0)
		void SetBox(double w, double h);

		void Translate(double x, double y) override;

		void Translate(Vec2 p) override;

		void Scale(double k) override;

		void Transform(SPC_Mat33 matrix) override;

		SPC_AABB ComputeAABB() const;

		void Combine(SPC_AABB aabb);

		SPC_AABB CombineAs(SPC_AABB aabb) const;

		bool Contains(Vec2 v) const;

		bool Contains(SPC_AABB aabb) const;

		SPC_AABB& operator=(const SPC_AABB& aabb);
	};

	SPC_AABB CombineAABB(SPC_AABB a, SPC_AABB b);

	double GetArea(SPC_AABB a);

	struct SPC_Circle : public SPC_Shape
	{
		double radius;
		Vec2 position;

		SPC_Circle(double r = 0, Vec2 p = Vec2(0, 0));

		SPC_Circle(const SPC_Circle& circle);

		SPC_Shape* Clone() const override;

		SPC_AABB ComputeAABB() const;

		void Translate(double x, double y) override;

		void Translate(Vec2 p) override;

		void Scale(double k) override;

		void Transform(SPC_Mat33 matrix) override;

		SPC_Circle& operator=(const SPC_Circle& circle);

	};

	struct SPC_Polygon : public SPC_Shape
	{
		static const int MAX_POLY = 64;
		Vec2 vertices[MAX_POLY];
		Vec2 normals[MAX_POLY];
		unsigned int vertexCount;

		SPC_Polygon();

		SPC_Polygon(const SPC_Polygon& polygon);

		SPC_Shape* Clone() const override;

		Vec2 GetCenter();

		//set polygon vertices from vector array. result is a convex hull. you can set vertices directly with ordering=false
		void Set(Vec2* v, unsigned int count, bool ordering = true);

		//set polygon vertices with size. top left is (0, 0)
		void SetBox(double w, double h);

		void Translate(double x, double y) override;

		void Translate(Vec2 p) override;

		void Scale(double k) override;

		void Rotate(double deg) override;
		 
		void Transform(SPC_Mat33 matrix) override;

		SPC_AABB ComputeAABB() const;

		SPC_Polygon& operator=(const SPC_Polygon& polygon);
	};
	
	SPC_AABB ComputeAABB(SPC_Shape* shape);
}
#endif