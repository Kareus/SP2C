#pragma once
#ifndef __SP_BODY__
#define __SP_BODY__

#include "SP_Vector.h"
#include "SP_Shapes.h"
#include "SP_MassData.h"
#include "SP_Material.h"

namespace SP2
{
	struct SP_Body
	{
		SP_Shape* shape;

		Vec2 force;
		Vec2 velocity;
		Vec2 position;

		SP_MassData massData;
		SP_Material material;

		unsigned int layer;

		double orientation;
		double angularVelocity;
		double torque;

		SP_Body();

		void ComputeMass();

		void ApplyForce(Vec2 f);

		void ApplyImpulse(Vec2 impulse, Vec2 contact);

		void AdjustForce();

	};
}
#endif