#include "test_def.h"

#if __MAIN_SPC__
#include <iostream>
#include <SFML/Graphics.hpp>

#include "SP/SP_Shapes.h"
#include "SP/SP_Engine.h"

using namespace std;
using namespace SP2;

pair<SP_Body*, SP_Shape*> createCircle(double r)
{
	SP_Body* body = new SP_Body;
	SP_Shape* shape = new SP_Circle(r);
	body->shape = shape;

	return { body, shape };
}

pair<SP_Body*, SP_Shape*> createBox(double w, double h)
{
	SP_Body* body = new SP_Body;
	SP_Polygon* shape = new SP_Polygon;
	shape->SetBox(w, h);
	body->shape = shape;

	return { body, shape };
}

int main()
{
	SP_Engine engine({ 0,0 }, 120.0);
	vector<SP_Body*> bodies;
	
	for (int i = 0; i < 32; i++)
	{
		int seed = rand() % 2;
		if (seed % 2)
		{
			auto p = createCircle(rand() % 100 + rand() % 100 * 0.01);
			auto body = p.first;
			body->material = Material::BouncyBall;
			body->ComputeMass();
			body->position = { (double)(rand() % 800), (double)(rand() % 600) };
			body->velocity = { (double)(rand() % 100) - 40, (double)(rand() % 100) - 40 };
			body->angularVelocity = rand() % 100 * 0.01 - 0.3;
			engine.AddBody(body);
			bodies.push_back(body);
		}
		else
		{
			auto p = createBox(rand() % 200 + rand() % 100 * 0.01, rand() % 200 + rand() % 100 * 0.01);
			auto body = p.first;
			body->material = Material::BouncyBall;
			body->ComputeMass();
			body->position = { (double)(rand() % 800), (double)(rand() % 600) };
			body->velocity = { (double)(rand() % 100) - 40, (double)(rand() % 100) - 40 };
			body->angularVelocity = (rand() % 100) * 0.01 - 0.3;
			body->orientation = (rand() % 100) * 0.01;
			engine.AddBody(body);
			bodies.push_back(body);
		}
	}

	sf::RenderWindow window(sf::VideoMode(1024, 768), "SP Test Window");

	sf::Clock clock;
	while (window.isOpen())
	{
		sf::Event e;
		while (window.pollEvent(e))
		{
			if (e.type == sf::Event::Closed)
				window.close();
		}

		double delta = clock.restart().asSeconds();
		engine.Update(delta);

		window.clear();
		
		for (auto& body : engine.getBodies())
		{
			SP_Shape* shape = body->shape;
			switch (shape->type)
			{
			case SP_Shape::Circle:
			{
				SP_Circle* c = reinterpret_cast<SP_Circle*>(shape);
				sf::CircleShape circle;
				circle.setRadius(c->radius);
				circle.setOrigin(c->radius, c->radius);
				circle.setPosition(body->position.x, body->position.y);
				circle.setFillColor(sf::Color::Transparent);
				circle.setOutlineColor(sf::Color::White);
				circle.setOutlineThickness(1);
				window.draw(circle);
			}
				break;

			case SP_Shape::Polygon:
			{
				SP_Polygon* p = reinterpret_cast<SP_Polygon*>(shape);
				sf::RectangleShape rect;
				rect.setSize(sf::Vector2f(p->vertices[1].x - p->vertices[0].x, p->vertices[2].y - p->vertices[1].y));
				rect.setOrigin(rect.getSize().x / 2, rect.getSize().y / 2);
				rect.setPosition(body->position.x, body->position.y);
				rect.setRotation(body->orientation * 180 / PI);
				rect.setFillColor(sf::Color::Transparent);
				rect.setOutlineColor(sf::Color::White);
				rect.setOutlineThickness(1);
				window.draw(rect);
			}
				break;

			default:
				break;
			}
		}

		window.display();
	}

	for (auto& body : bodies)
	{
		delete body->shape;
		delete body;
	}

	bodies.clear();
	return 0;
}
#endif