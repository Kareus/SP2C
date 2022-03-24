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

		void __IntegrateForce(SP_Body* body, double delta)
		{
			if (body->massData.inv_mass == 0) return;
			body->velocity += (body->massData.inv_mass * body->force + gravity) * delta / 2; //Symplectric Euler
			body->angularVelocity += body->torque * body->massData.inv_inertia * delta / 2;
		}

		void __IntegrateVelocity(SP_Body* body, double delta)
		{
			if (body->massData.inv_mass == 0) return;
			body->position += body->velocity * delta;
			body->orientation += body->angularVelocity * delta;

			__IntegrateForce(body, delta);
		}

		void __Update(double delta)
		{
			contacts.clear();

			int S = bodies.size();
			for (int i = 0; i < S; i++)
			{
				SP_Body* A = bodies[i];
				for (int j = i + 1; j < S; j++)
				{
					SP_Body* B = bodies[j];
					if (A->massData.inv_mass == 0 && B->massData.inv_mass == 0) continue;

					SP_Manifold manifold(A, B);
					bool collided = CollideTest[A->shape->type][B->shape->type](&manifold);

					if (collided) contacts.emplace_back(manifold);
				}
			}

			for (auto body : bodies)
			{
				__IntegrateForce(body, delta);
				body->AdjustForce();
			}

			for (auto& contact : contacts)
				contact.ApplyImpulse();

			for (auto& body : bodies)
			{
				__IntegrateVelocity(body, delta);
				body->AdjustForce();
			}

			for (auto& contact : contacts)
				CorrectPosition(&contact);

			for (auto& body : bodies)
			{
				body->force = { 0,0 };
				body->torque = 0;
			}
		}

	public:
		SP_Engine(Vec2 g = Vec2(0, 9.8), double fps = 120) : FPS(fps), gravity(g)
		{
			dt = 1 / fps;
		};

		void AddBody(SP_Body* body)
		{
			bodies.push_back(body);
		}

		void RemoveBody(size_t index)
		{
			bodies.erase(bodies.begin() + index);
		}

		void RemoveBody(SP_Body* body)
		{
			int i, S = bodies.size();
			for (i = 0; i < S; i++)
				if (bodies[i] == body) break;

			if (i < S) bodies.erase(bodies.begin() + i);
		}

		void Update(double delta)
		{
			accumulator += delta;
			if (accumulator > acc_limit)
				accumulator = acc_limit;

			while (accumulator >= dt)
			{
				__Update(dt);
				accumulator -= dt;
			}
		}

		const std::vector<SP_Body*>& getBodies() const
		{
			return bodies;
		}
	};
}
#endif