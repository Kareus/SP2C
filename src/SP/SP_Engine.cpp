#include "SP_Engine.h"

namespace SP2
{
	void SP_Engine::__IntegrateForce(SP_Body* body, double delta)
	{
		if (body->massData.inv_mass == 0) return;
		body->velocity += (body->massData.inv_mass * body->force + gravity) * delta / 2; //Symplectric Euler
		body->angularVelocity += body->torque * body->massData.inv_inertia * delta / 2;
	}

	void SP_Engine::__IntegrateVelocity(SP_Body* body, double delta)
	{
		if (body->massData.inv_mass == 0) return;
		body->position += body->velocity * delta;
		body->orientation += body->angularVelocity * delta;

		__IntegrateForce(body, delta);
	}

	void SP_Engine::__Update(double delta)
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

	SP_Engine::SP_Engine(Vec2 g, double fps) : FPS(fps), gravity(g)
	{
		dt = 1 / fps;
	};

	void SP_Engine::AddBody(SP_Body* body)
	{
		bodies.push_back(body);
	}

	void SP_Engine::RemoveBody(size_t index)
	{
		bodies.erase(bodies.begin() + index);
	}

	void SP_Engine::RemoveBody(SP_Body* body)
	{
		int i, S = bodies.size();
		for (i = 0; i < S; i++)
			if (bodies[i] == body) break;

		if (i < S) bodies.erase(bodies.begin() + i);
	}

	void SP_Engine::Update(double delta)
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

	const std::vector<SP_Body*>& SP_Engine::getBodies() const
	{
		return bodies;
	}
}