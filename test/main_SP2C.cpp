#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>
#include <map>

#include <SP2C/SPC_Shapes.h>
#include <SP2C/SPC_Collision.h>

using namespace std;
using namespace SP2C;
using namespace SP2C::Const;

#define ROTATE_POLYGON_TEST 0

struct ShapeTester
{
	vector<SPC_Shape*> shapes;
	bool collided = false;
	Vec2 velocity;
	int type;
	double w; //angular velocity for polygon
};

SPC_Circle* createCircle(double r)
{
	return new SPC_Circle(r);
}

SPC_AABB* createBox(double w, double h)
{
	return new SPC_AABB(Vec2(-w / 2, -h / 2), Vec2(w / 2, h / 2));
}

SPC_Polygon* createPolygon(vector<Vec2> v)
{
	Vec2 vertices[SPC_Polygon::MAX_POLY];

	int s = 0;
	for (auto& p : v)
	{
		vertices[s++] = p;
		if (s >= SPC_Polygon::MAX_POLY) break;
	}

	SPC_Polygon* polygon = new SPC_Polygon;
	polygon->Set(vertices, s);
	return polygon;
}

vector<SPC_Shape*> createRoundRect(double w, double h, double r)
{
	vector<SPC_Shape*> ret;

	if (r == 0)
	{
		ret.push_back(new SPC_AABB(Vec2(-w / 2, -h / 2), Vec2(w / 2, h / 2)));
		return ret;
	}

	if (w / 2 <= r && h / 2 <= r)
	{
		ret.push_back(new SPC_Circle(r));
		return ret;
	}
	else if (w / 2 <= r)
	{
		SPC_Circle* r1 = new SPC_Circle(r);
		r1->position = Vec2(0, r - h / 2);
		SPC_AABB* box = new SPC_AABB(Vec2(-r, r - h / 2), Vec2(r, h / 2 - r));
		SPC_Circle* r2 = new SPC_Circle(r);
		r2->position = Vec2(0, h / 2 - r);

		ret.push_back(r1);
		ret.push_back(box);
		ret.push_back(r2);
		return ret;
	}
	else if (h / 2 <= r)
	{
		SPC_Circle* r1 = new SPC_Circle(r);
		r1->position = Vec2(r - w / 2, 0);
		SPC_AABB* box = new SPC_AABB(Vec2(r - w / 2, -r), Vec2(w / 2 - r, r));
		SPC_Circle* r2 = new SPC_Circle(r);
		r2->position = Vec2(w / 2 - r, 0);

		ret.push_back(r1);
		ret.push_back(box);
		ret.push_back(r2);
		return ret;
	}
	else
	{
		SPC_Circle* r1 = new SPC_Circle(r);
		r1->position = Vec2(r - w / 2, r - h / 2);
		SPC_Circle* r2 = new SPC_Circle(r);
		r2->position = Vec2(w / 2 - r, r - h / 2);

		SPC_AABB* box1 = new SPC_AABB(Vec2(r - w / 2, -h / 2), Vec2(w / 2 - r, h / 2));
		SPC_AABB* box2 = new SPC_AABB(Vec2(-w / 2, r - h / 2), Vec2(w / 2, h / 2 - r));

		SPC_Circle* r3 = new SPC_Circle(r);
		r3->position = Vec2(w / 2 - r, h / 2 - r);
		SPC_Circle* r4 = new SPC_Circle(r);
		r4->position = Vec2(r - w / 2, h / 2 - r); //clockwise order

		ret.push_back(r1);
		ret.push_back(r2);
		ret.push_back(box1);
		ret.push_back(box2);
		ret.push_back(r3);
		ret.push_back(r4);

		return ret;
	}
}

vector<SPC_Shape*> triangulation(const vector<Vec2>& v) //polygon triangulation using ear clipping
{
	if (v.size() < 3) return {};

	Vec2 triangle[3];
	vector<SPC_Shape*> ret;

	if (v.size() == 3)
	{
		triangle[0] = v[0], triangle[1] = v[1], triangle[2] = v[2];
		SPC_Polygon* polygon = new SPC_Polygon;
		polygon->Set(triangle, 3);
		ret.push_back(polygon);
		return ret;
	}

	double area = 0;
	int N = v.size();

	for (int i = 0; i < N; i++)
		area += CrossProduct(v[(i + 1) % N], v[i]);

	vector<int> idx;
	vector<int> cnt;
	map<pair<int, int>, int> edge;
	cnt.resize(N, 0);

	if (area < 0)
		for (int i = 0; i < N; i++) idx.push_back(i);
	else
		for (int i = 0; i < N; i++) idx.push_back(N - i - 1);

	for (int j = N - 1; N > 2;)
	{

		int i = j % N;
		j = (i + 1) % N;
		int k = (j + 1) % N;

		bool ear = CrossProduct(v[idx[k]] - v[idx[i]], v[idx[j]] - v[idx[i]]) < 0;

		for (int t = 0; ear && t < idx.size(); t++)
			if (t == i || t == j || t == k) continue;
			else ear = !InsideTriangle(v[idx[i]], v[idx[j]], v[idx[k]], v[idx[t]]);

		if (ear)
		{
			triangle[0] = v[idx[i]], triangle[1] = v[idx[j]], triangle[2] = v[idx[k]];
			pair<int, int> ab = { min(idx[i], idx[j]), max(idx[i], idx[j]) }, bc = { min(idx[j], idx[k]), max(idx[j], idx[k]) }, ca = { min(idx[k], idx[i]), max(idx[k], idx[i]) };

			edge[ab]++, edge[bc]++, edge[ca]++;
			SPC_Polygon* polygon = new SPC_Polygon;
			polygon->Set(triangle, 3, false); //to make outline drawing simple
			ret.push_back(polygon);

			for (int t = j; t < N - 1; t++) idx[t] = idx[t + 1];

			N--;
		}

	}

	for (auto& p : edge)
		if (p.second == 1) cnt[p.first.first]++, cnt[p.first.second]++;

	int num = 0;
	N = v.size();
	for (int i = 0; i < N; i++)
		if (cnt[i])
		{
			num++;
			cout << i << ' ';
		}

	cout << ": " << num << "," << N << endl;
	return ret;
}

void drawRoundRect(sf::RenderTarget& target, vector<SPC_Shape*>& shapes, sf::Color color)
{
	if (shapes.size() == 1) //only one circle or one aabb (actually, we don't reach here)
	{
		switch (shapes[0]->type)
		{ 
			case SPC_Shape::AABB:
			{
				SPC_AABB* a = reinterpret_cast<SPC_AABB*>(shapes[0]);
				sf::RectangleShape rect;
				auto extent = a->GetExtent();
				auto pos = a->GetCenter();
				rect.setSize(sf::Vector2f(extent.x * 2, extent.y * 2));
				rect.setOrigin(sf::Vector2f(extent.x, extent.y));
				rect.setPosition(pos.x, pos.y);
				rect.setFillColor(sf::Color::Transparent);
				rect.setOutlineColor(color);
				rect.setOutlineThickness(1);
				target.draw(rect);

				return;
			}

			case SPC_Shape::Circle:
			{
				SPC_Circle* c = reinterpret_cast<SPC_Circle*>(shapes[0]);
				sf::CircleShape circle;
				circle.setRadius(c->radius);
				circle.setOrigin(c->radius, c->radius);
				circle.setPosition(c->position.x, c->position.y);
				circle.setFillColor(sf::Color::Transparent);
				circle.setOutlineColor(color);
				circle.setOutlineThickness(1);
				target.draw(circle);
				return;
			}

			default:
				return;
		}
	}

	static const int roundrect_mode = 2;

	if (roundrect_mode == 1) //method 1 : draw each shapes
	{
		for (auto& shape : shapes)
		{
			switch (shape->type)
			{
			case SPC_Shape::AABB:
			{
				SPC_AABB* a = reinterpret_cast<SPC_AABB*>(shape);
				sf::RectangleShape rect;
				auto extent = a->GetExtent();
				auto pos = a->GetCenter();
				rect.setSize(sf::Vector2f(extent.x * 2, extent.y * 2));
				rect.setOrigin(sf::Vector2f(extent.x, extent.y));
				rect.setPosition(pos.x, pos.y);
				rect.setFillColor(sf::Color::Transparent);
				rect.setOutlineColor(color);
				rect.setOutlineThickness(1);
				target.draw(rect);
				break;
			}

			case SPC_Shape::Circle:
			{
				SPC_Circle* c = reinterpret_cast<SPC_Circle*>(shape);
				sf::CircleShape circle;
				circle.setRadius(c->radius);
				circle.setOrigin(c->radius, c->radius);
				circle.setPosition(c->position.x, c->position.y);
				circle.setFillColor(sf::Color::Transparent);
				circle.setOutlineColor(color);
				circle.setOutlineThickness(1);
				target.draw(circle);
				break;
			}
			}
		}
	}
	else //method 2 : draw outline
	{
		sf::VertexArray arr(sf::LinesStrip);
		const double theta = 10 * RAD;
		double angle = 0;
		sf::Vector2f p;

		if (shapes.size() == 3)
		{
			SPC_Circle* c1 = reinterpret_cast<SPC_Circle*>(shapes[0]);
			SPC_Circle* c2 = reinterpret_cast<SPC_Circle*>(shapes[2]);

			if (c1->position.x == c2->position.x)
			{
				angle = -PI;
				do
				{
					p.x = c1->position.x + c1->radius * cos(angle);
					p.y = c1->position.y + c1->radius * sin(angle);
					arr.append(p);
					angle += theta;
				} while (angle <= 0);

				angle = 0;
				do
				{
					p.x = c2->position.x + c2->radius * cos(angle);
					p.y = c2->position.y + c2->radius * sin(angle);
					arr.append(p);
					angle += theta;
				} while (angle <= PI);

				arr.append(sf::Vector2f(c1->position.x - c1->radius, c1->position.y));
			}
			else
			{
				angle = -PI * 3 / 2;
				do
				{
					p.x = c1->position.x + c1->radius * cos(angle);
					p.y = c1->position.y + c1->radius * sin(angle);
					arr.append(p);
					angle += theta;
				} while (angle <= -PI / 2);

				angle = -PI / 2;
				do
				{
					p.x = c2->position.x + c2->radius * cos(angle);
					p.y = c2->position.y + c2->radius * sin(angle);
					arr.append(p);
					angle += theta;
				} while (angle <= PI / 2);

				arr.append(sf::Vector2f(c1->position.x, c1->position.y + c1->radius));
			}
		}
		else
		{
			SPC_Circle* c1 = reinterpret_cast<SPC_Circle*>(shapes[0]);
			SPC_Circle* c2 = reinterpret_cast<SPC_Circle*>(shapes[1]);
			SPC_Circle* c3 = reinterpret_cast<SPC_Circle*>(shapes[4]);
			SPC_Circle* c4 = reinterpret_cast<SPC_Circle*>(shapes[5]);

			angle = -PI;
			do
			{
				p.x = c1->position.x + c1->radius * cos(angle);
				p.y = c1->position.y + c1->radius * sin(angle);
				arr.append(p);
				angle += theta;
			} while (angle <= -PI / 2);

			angle = -PI / 2;
			do
			{
				p.x = c2->position.x + c2->radius * cos(angle);
				p.y = c2->position.y + c2->radius * sin(angle);
				arr.append(p);
				angle += theta;
			} while (angle <= 0);

			angle = 0;
			do
			{
				p.x = c3->position.x + c3->radius * cos(angle);
				p.y = c3->position.y + c3->radius * sin(angle);
				arr.append(p);
				angle += theta;
			} while (angle <= PI / 2);

			angle = PI / 2;
			do
			{
				p.x = c4->position.x + c4->radius * cos(angle);
				p.y = c4->position.y + c4->radius * sin(angle);
				arr.append(p);
				angle += theta;
			} while (angle <= PI);

			arr.append(sf::Vector2f(c1->position.x - c1->radius, c1->position.y));
		}

		int s = arr.getVertexCount();
		for (int i = 0; i < s; i++)
			arr[i].color = color;

		target.draw(arr);
	}
}

void drawTriangles(sf::RenderTarget& target, vector<SPC_Shape*>& shapes, sf::Color color)
{
	sf::Color cc[3] = { sf::Color::Red, sf::Color::Green, sf::Color::Blue };
	
	static const int triangle_mode = 1;

	if (triangle_mode == 1)
	{
		//method 1: draw each triangles
		for (auto& shape : shapes)
		{
			SPC_Polygon* p = reinterpret_cast<SPC_Polygon*>(shape);

			sf::VertexArray arr(sf::LinesStrip); //convex would result in some errors due to some points not in correct order
			for (int i = 0; i < 3; i++)
			{
				arr.append(sf::Vector2f(p->vertices[i].x, p->vertices[i].y));
				arr[i].color = color;
			}

			arr.append(sf::Vector2f(p->vertices[0].x, p->vertices[0].y));
			arr[3].color = color;

			target.draw(arr);
		}
	}
	else
	{
		//method:2 draw outline
		sf::VertexArray arr(sf::LinesStrip);

		vector<Vec2> v;
		//well, I think you can simply get original vertex data used in trangulation and draw it.

		for (int i = 0; i < shapes.size(); i++)
		{
			SPC_Polygon* p = reinterpret_cast<SPC_Polygon*>(shapes[i]);

			for (int j = 0; j < 3; j++)
			{
				bool found = false;

				for (auto& pt : v)
					if (p->vertices[j] == pt)
					{
						found = true;
						break;
					}


				if (!found) v.push_back(p->vertices[j]);
			}
		}

		v.push_back(v[0]); //close the outline

		for (int s = 0; s < v.size(); s++)
		{
			arr.append(sf::Vector2f(v[s].x, v[s].y));
			arr[s].color = color;
		}

		target.draw(arr);
	}
}

void drawVertices(sf::RenderTarget& target, vector<Vec2>& shapes, sf::Color color)
{
	sf::VertexArray arr(sf::LinesStrip);
	for (auto& v : shapes)
		arr.append(sf::Vector2f(v.x, v.y));

	arr.append(sf::Vector2f(shapes[0].x, shapes[0].y));

	int s = arr.getVertexCount();
	for (int i = 0; i < s; i++)
		arr[i].color = color;

	target.draw(arr);
}

int main()
{
	vector<ShapeTester> shapes;

	ShapeTester tester;

	for (int i = 0; i < 50; i++) //create shapes
	{
		int seed = rand() % 5;
		tester.type = seed;
		tester.w = 0;

		if (seed == 0) //circle
		{
			auto circle = createCircle(rand() % 100 + rand() % 100 * 0.01);

			circle->position = { (double)(rand() % 800), (double)(rand() % 600) };
			
			tester.shapes = { circle };
			tester.velocity = Vec2((double)(rand() % 100 - 50), (double)(rand() % 100 - 50));
			shapes.push_back(tester);
		}
		else if (seed == 1) //aabb (box)
		{
			auto box = createBox(rand() % 200 + rand() % 100 * 0.01, rand() % 200 + rand() % 100 * 0.01);
			box->Translate((double)(rand() % 800), (double)(rand() % 600));
			
			tester.shapes = { box };
			tester.velocity = Vec2((double)(rand() % 100 - 50), (double)(rand() % 100 - 50));
			shapes.push_back(tester);
		}
		else if (seed == 2) //polygon (convex)
		{
			int count = rand() % 12 + 3; //vertex count
			vector<Vec2> v;
			Vec2 pos = { (double)(rand() % 800), (double)(rand() % 600) };

			for (int i = 0; i < count; i++)
				v.emplace_back(pos.x + (double)(rand() % 200) - 100, pos.y + (double)(rand() % 200) - 100);

			auto polygon = createPolygon(v);
			
			tester.shapes = { polygon };
			tester.velocity = Vec2((double)(rand() % 100 - 50), (double)(rand() % 100 - 50));

#if ROTATE_POLYGON_TEST
			tester.w = (double)(rand() % 30 + rand() % 100 * 0.01 - 15);
#endif
			shapes.push_back(tester);
		}
		else if (seed == 3) //roundrect (custom shape. I defined using a group of circles and aabbs)
		{
			double w = rand() % 200 + rand() % 100 * 0.01, h = rand() % 200 + rand() % 100 * 0.01;
			int r = (int)(max(w, h) / 2);
			auto roundrect = createRoundRect(w, h, rand() % r);
			Vec2 pos = { (double)(rand() % 800), (double)(rand() % 600) };

			for (auto& s : roundrect)
				if (s->type == SPC_Shape::AABB)
					reinterpret_cast<SPC_AABB*>(s)->Translate(pos.x, pos.y);
				else
					reinterpret_cast<SPC_Circle*>(s)->position += pos;

			tester.shapes = roundrect;
			tester.velocity = Vec2((double)(rand() % 100 - 50), (double)(rand() % 100 - 50));
			shapes.push_back(tester);
		}
		else if (seed == 4) //polygon (can be concave)
		{
			int count = rand() % 12 + 3; //vertex count
			vector<Vec2> v;
			Vec2 pos = { (double)(rand() % 800), (double)(rand() % 600) };

			double angle = 0, theta = 2 * PI / count;
			for (int i = 0; i < count; i++)
			{
				double r = rand() % 90 + rand() % 100 * 0.01 + 10;
				v.emplace_back(pos.x + r * cos(angle), pos.y + r * sin(angle));
				angle += theta;
			}
			
			auto triangles = triangulation(v);

			tester.shapes = { triangles };
			tester.velocity = Vec2((double)(rand() % 100 - 50), (double)(rand() % 100 - 50));
#if ROTATE_POLYGON_TEST
			tester.w = 0; //pivot should be the center of group, not each centers. didn't implement here.
#endif
			shapes.push_back(tester);
		}
	}
	
	/*
	SP2C::SPC_AABB aabb;
	aabb.SetBox(50, 50);
	aabb.Translate(300, 100);

	SP2C::SPC_Polygon polygon;
	polygon.SetBox(200, 200);
	polygon.Translate(300, 300);

	SP2C::SPC_Circle circle;
	circle.radius = 50;
	circle.position = Vec2(145, 280);
	//circle.position = Vec2(100, 300);

	tester.shapes = { &aabb };
	shapes.push_back(tester);

	tester.shapes = { &polygon };
	shapes.push_back(tester);

	tester.shapes = { &circle };
	//tester.velocity = Vec2(50, 0);
	shapes.push_back(tester);
	*/

	SPC_Manifold m;

	sf::RenderWindow window(sf::VideoMode(1024, 768), "SPC Test Window");

	sf::Clock clock;

	sf::Font font;
	font.loadFromFile("Arial.ttf");

	sf::Text fpsCounter;
	fpsCounter.setFont(font);
	fpsCounter.setFillColor(sf::Color::White);
	fpsCounter.setCharacterSize(18);

	double fpsTime = 0;
	double scaleValue = 1;
	int fpsCount = 0, fps = 0;
	int keycheck = 0;

	while (window.isOpen()) //window
	{
		sf::Event e;
		while (window.pollEvent(e))
		{
			if (e.type == sf::Event::Closed)
				window.close();
		}

		double delta = clock.restart().asSeconds();
		fpsTime += delta;

		if (fpsTime >= 1)
		{
			fpsTime = 0;
			fps = fpsCount;
			fpsCounter.setString("FPS: " + to_string(fps));
			fpsCount = 0;
		}

		fpsCount++;
		for (int i = 0; i < shapes.size(); i++) //reset
			shapes[i].collided = false;

		window.clear();

		/*
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) keycheck++;
		if (!keycheck)
		{
			window.display();
			continue;
		}
		*/

		//matrix.m[0][0] = scaleValue;
		//scaleValue -= 0.1 * delta;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) shapes[0].shapes[0]->Translate(0, -100 * delta);
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) shapes[0].shapes[0]->Translate(0, 100 * delta);

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) shapes[0].shapes[0]->Translate(-100 * delta, 0);
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) shapes[0].shapes[0]->Translate(100 * delta, 0);
		
		for (int i = 0; i < shapes.size(); i++) //collide test
			for (int j = i + 1; j < shapes.size(); j++)
			{
				bool check = false;
				for (int ii = 0; ii < shapes[i].shapes.size() && !check; ii++)
				{
					m.A = shapes[i].shapes[ii]->Clone();
					//m.A->Transform(matrix);

					for (int jj = 0; jj < shapes[j].shapes.size() && !check; jj++)
					{
						m.B = shapes[j].shapes[jj]->Clone();
						//m.B->Transform(matrix);

						if (Collision::Collide(&m))
						{
							//shapes[i].velocity = Vec2(0, 0);
							//shapes[j].velocity = Vec2(0, 0);

							shapes[i].collided = shapes[j].collided = true;
							check = true;

							/*
							for (auto& shape : shapes[i].shapes)
								shape->Translate(-m.normal * m.penetration * delta);

							for (auto& shape : shapes[j].shapes)
								shape->Translate(m.normal * m.penetration * delta);
								*/

							sf::VertexArray arr(sf::Lines);
							sf::Vector2f contact = sf::Vector2f(m.contact_points[0].x, m.contact_points[0].y);
							arr.append(contact);
							arr.append(contact + sf::Vector2f(m.normal.x * m.penetration, m.normal.y * m.penetration));
							arr[0].color = sf::Color::Green;
							arr[1].color = sf::Color::Blue;

							window.draw(arr);
						}

						delete m.B;
					}

					delete m.A;
				}
			}
		
		for (auto& test : shapes) //draw shapes
		{
			sf::Color color = test.collided ? sf::Color::Red : sf::Color::White;

			auto& shape = test.shapes;
			if (shape.size() == 1)
			{
				switch (shape[0]->type)
				{
					case SPC_Shape::AABB:
					{
						SPC_AABB* a = reinterpret_cast<SPC_AABB*>(shape[0]->Clone());
						//a->Transform(matrix);
						sf::RectangleShape rect;
						auto extent = a->GetExtent();
						auto pos = a->GetCenter();
						rect.setSize(sf::Vector2f(extent.x * 2, extent.y * 2));
						rect.setOrigin(sf::Vector2f(extent.x, extent.y));
						rect.setPosition(pos.x, pos.y);
						rect.setFillColor(sf::Color::Transparent);
						rect.setOutlineColor(color);
						rect.setOutlineThickness(1);
						window.draw(rect);

						delete a;
						break;
					}

					case SPC_Shape::Circle:
					{
						SPC_Circle* c = reinterpret_cast<SPC_Circle*>(shape[0]->Clone());
						//c->Transform(matrix);
						sf::CircleShape circle;
						circle.setRadius(c->radius);
						circle.setOrigin(c->radius, c->radius);
						circle.setPosition(c->position.x, c->position.y);
						circle.setFillColor(sf::Color::Transparent);
						circle.setOutlineColor(color);
						circle.setOutlineThickness(1);
						window.draw(circle);

						delete c;
						break;
					}

					case SPC_Shape::Polygon:
					{
						SPC_Polygon* p = reinterpret_cast<SPC_Polygon*>(shape[0]->Clone());
						//p->Transform(matrix);
						sf::ConvexShape cv;
						cv.setPointCount(p->vertexCount);

						for (int i = 0; i < p->vertexCount; i++)
							cv.setPoint(i, sf::Vector2f(p->vertices[i].x, p->vertices[i].y));

						cv.setFillColor(sf::Color::Transparent);
						cv.setOutlineColor(color);
						cv.setOutlineThickness(1);
						window.draw(cv);

						delete p;
						break;
					}

					default:
					break;
				}
			}
			else if (test.type == 3)
				drawRoundRect(window, shape, color); //roundrect is actually a group of circles and boxes, so we draw using vertex array.
			else if (test.type == 4)
				drawTriangles(window, shape, color); //also, a group of triangles.
		}

		window.draw(fpsCounter);
		window.display();
		//continue;

		for (auto& shape : shapes) //move shapes
		{
			for (auto& s : shape.shapes)
			{
				switch (s->type)
				{
				case SPC_Shape::AABB:
					reinterpret_cast<SPC_AABB*>(s)->Translate(shape.velocity.x * delta, shape.velocity.y * delta);
					break;

				case SPC_Shape::Circle:
					reinterpret_cast<SPC_Circle*>(s)->position += shape.velocity * delta;
					break;

				case SPC_Shape::Polygon:
				{
					auto polygon = reinterpret_cast<SPC_Polygon*>(s);
					polygon->Translate(shape.velocity.x * delta, shape.velocity.y * delta);

#if ROTATE_POLYGON_TEST
					polygon->Rotate(shape.w * delta);
#endif
					break;
				}

				default:
					break;
				}
			}
		}
	}

	for (auto& shape : shapes) //destroy
	{
		for (auto& s : shape.shapes)
			delete s;

		shape.shapes.clear();
	}

	shapes.clear();
	return 0;
}