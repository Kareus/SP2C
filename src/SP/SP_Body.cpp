#include "SP_Body.h"

namespace SP2
{
	SP_Body::SP_Body()
	{
		shape = nullptr;
		force = velocity = position = { 0,0 };
		layer = 0;
		massData = SP_MassData();
		material = SP_Material();

		orientation = 0;
		angularVelocity = 0;
		torque = 0;
	}

	void SP_Body::ComputeMass()
	{
		if (shape == nullptr) return;

		switch (shape->type)
		{
		case SP_Shape::Circle:
		{
			SP_Circle* circle = reinterpret_cast<SP_Circle*>(shape);
			massData.mass = PI * circle->radius * circle->radius * material.density;
			massData.inv_mass = massData.mass == 0 ? 0 : (1.0 / massData.mass);
			massData.inertia = massData.mass * circle->radius * circle->radius;
			massData.inv_inertia = massData.inertia == 0 ? 0 : (1.0 / massData.inertia);
		}
		break;

		case SP_Shape::Polygon:
		{
			SP_Polygon* polygon = reinterpret_cast<SP_Polygon*>(shape);
			double area = 0;
			double I = 0;
			const double k_inv3 = 1.0 / 3.0;

			for (unsigned int i = 0; i < polygon->vertexCount; i++)
			{
				Vec2 p1(polygon->vertices[i].x, polygon->vertices[i].y);
				unsigned int i2 = i + 1 < polygon->vertexCount ? i + 1 : 0;
				Vec2 p2(polygon->vertices[i2].x, polygon->vertices[i2].y);

				double D = CrossProduct(p1, p2);
				double triangleArea = 0.5 * D;
				area += triangleArea;

				double intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
				double inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
				I += (0.25 * k_inv3 * D) * (intx2 + inty2);
			}

			massData.mass = material.density * area;
			massData.inv_mass = massData.mass == 0 ? 0 : (1.0 / massData.mass);
			massData.inertia = I * material.density;
			massData.inv_inertia = massData.inertia = 00 ? 0 : (1.0 / massData.inertia);
		}
		break;

		default:
			massData.mass = massData.inv_mass = 0;
			massData.inertia = massData.inv_inertia = 0;
			break;
		}
	}

	void SP_Body::ApplyForce(Vec2 f)
	{
		force += f;
	}

	void SP_Body::ApplyImpulse(Vec2 impulse, Vec2 contact)
	{
		velocity += massData.inv_mass * impulse;
		angularVelocity += massData.inv_inertia * CrossProduct(contact, impulse);
		AdjustForce();
	}

	void SP_Body::AdjustForce()
	{
		if (angularVelocity >= 2 * PI) angularVelocity -= 2 * PI * (int)(angularVelocity / (2 * PI));
		if (angularVelocity <= -2 * PI)  angularVelocity += 2 * PI * (int)(-angularVelocity / (2 * PI));
		if (orientation >= 2 * PI) orientation -= (int)(orientation / (2 * PI)) * 2 * PI;
		if (orientation <= -2 * PI) orientation += (int)(-orientation / (2 * PI)) * 2 * PI;
	}
}