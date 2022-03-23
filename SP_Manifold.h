#pragma once
#ifndef __SP_MANIFOLD__
#define __SP_MANIFOLD__

#include "SP_Body.h"

namespace SP2
{
	struct SP_Manifold
	{
		SP_Body* A;
		SP_Body* B;
		Vec2 normal;
		double penetration;
		unsigned int contact_count;
		Vec2 contacts[2];

		double e;
		double friction_static;
		double friction_dynamic;

		SP_Manifold(SP_Body* a = nullptr, SP_Body* b = nullptr) : A(a), B(b)
		{
			Initialize();
		};

		void Initialize()
		{
			if (!A || !B) return;
			e = std::min(A->material.restitution, B->material.restitution);
			friction_static = Pythagorean(A->material.friction_static, B->material.friction_static);
			friction_dynamic = Pythagorean(A->material.friction_dynamic, B->material.friction_dynamic);
		}

		void Initialize(double dt, Vec2 g)
		{
			Initialize();

			for (unsigned int i = 0; i < contact_count; i++)
			{
				Vec2 ra = contacts[i] - A->position;
				Vec2 rb = contacts[i] - B->position;

				Vec2 rv = B->velocity + CrossProduct(B->angularVelocity, rb) - A->velocity - CrossProduct(A->angularVelocity, ra);

				if (rv.LengthSquared() < (dt * g).LengthSquared() + 1e-4)
					e = 0;
			}
		}

		void ApplyImpulse()
		{
			if (std::abs(A->massData.inv_mass + B->massData.inv_mass) <= 1e-4)
			{
				A->velocity = { 0,0 };
				B->velocity = { 0,0 };
				return;
			}

			for (unsigned int i = 0; i < contact_count; i++)
			{
				Vec2 ra = contacts[i] - A->position;
				Vec2 rb = contacts[i] - B->position;

				Vec2 rv = B->velocity + CrossProduct(B->angularVelocity, rb) - A->velocity - CrossProduct(A->angularVelocity, ra);

				double contactVelocity = DotProduct(rv, normal);

				if (contactVelocity > 0) return;

				double raCrossN = CrossProduct(ra, normal);
				double rbCrossN = CrossProduct(rb, normal);
				double invMassSum = A->massData.inv_mass + B->massData.inv_mass + raCrossN * raCrossN * A->massData.inv_inertia + rbCrossN * rbCrossN * B->massData.inv_inertia;

				double scalar = -(e + 1) * contactVelocity;
				scalar /= invMassSum;
				scalar /= contact_count;

				Vec2 impulse = normal * scalar;
				A->ApplyImpulse(-impulse, ra);
				B->ApplyImpulse(impulse, rb);

				rv = B->velocity + CrossProduct(B->angularVelocity, rb) - A->velocity - CrossProduct(A->angularVelocity, ra);

				Vec2 tangent = rv - (normal * DotProduct(rv, normal));
				tangent.Normalize();

				double scalar_t = -DotProduct(rv, tangent);
				scalar_t /= invMassSum;
				scalar_t /= contact_count;

				if (std::abs(scalar_t) <= 1e-4) return;

				Vec2 tangentImpulse;
				if (std::abs(scalar_t) < scalar * friction_static)
					tangentImpulse = tangent * scalar_t;
				else
					tangentImpulse = tangent * -scalar * friction_dynamic;

				A->ApplyImpulse(-tangentImpulse, ra);
				B->ApplyImpulse(tangentImpulse, rb);
			}
		}
	};
}
#endif