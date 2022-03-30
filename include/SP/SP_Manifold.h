#pragma once
#ifndef __SP_MANIFOLD__
#define __SP_MANIFOLD__

#include <cmath>
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

		SP_Manifold(SP_Body* a = nullptr, SP_Body* b = nullptr);

		void Initialize();

		void Initialize(double dt, Vec2 g);

		void ApplyImpulse();
	};
}
#endif