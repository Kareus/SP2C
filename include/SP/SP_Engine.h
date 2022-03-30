#pragma once
#ifndef __SP_ENGINE__
#define __SP_ENGINE__

#include <vector>
#include <iostream>
#include "SP_Physics.h"

namespace SP2
{
	class SP_Engine
	{
		std::vector<SP_Body*> bodies;
		std::vector<SP_Manifold> contacts;
		const double acc_limit = 0.2;
		double FPS, dt;
		double accumulator = 0;
		Vec2 gravity;

		void __IntegrateForce(SP_Body* body, double delta);

		void __IntegrateVelocity(SP_Body* body, double delta);

		void __Update(double delta);

	public:
		SP_Engine(Vec2 g = Vec2(0, 9.8), double fps = 120);

		void AddBody(SP_Body* body);

		void RemoveBody(size_t index);

		void RemoveBody(SP_Body* body);

		void Update(double delta);

		const std::vector<SP_Body*>& getBodies() const;
	};
}
#endif