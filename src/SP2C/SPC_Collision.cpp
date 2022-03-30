#include <SP2C/SPC_Collision.h>

namespace SP2C
{
	namespace Collision
	{
		int Clip(Vec2 n, double c, Vec2* face)
		{
			unsigned int sp = 0;
			Vec2 out[2] = { face[0], face[1] };

			double d1 = DotProduct(n, face[0]) - c;
			double d2 = DotProduct(n, face[1]) - c;

			if (d1 <= 0) out[sp++] = face[0];
			if (d2 <= 0) out[sp++] = face[1];

			if (d1 * d2 < 0)
			{
				double alpha = d1 / (d1 - d2);
				out[sp++] = face[0] + alpha * (face[1] - face[0]);
			}

			face[0] = out[0];
			face[1] = out[1];

			assert(sp != 3);
			return sp;
		}

		double FindAxisLeastPenetration(unsigned int* faceIndex, Vec2* a, Vec2* normals, unsigned int n, Vec2* b, unsigned int m)
		{
			double bestDistance = -DBL_MAX;
			unsigned int bestIndex = 0;

			for (unsigned int i = 0; i < n; i++)
			{
				double d = DotProduct(normals[i], GetSupport(-normals[i], b, m) - a[i]);

				if (d > bestDistance)
				{
					bestDistance = d;
					bestIndex = i;
				}
			}

			*faceIndex = bestIndex;
			return bestDistance;
		}

		void FindIncidentFace(Vec2* v, Vec2 refNormal, Vec2* incVertices, Vec2* incNormals, unsigned int n)
		{
			int incidentFace = 0;
			double minDot = DBL_MAX;
			for (unsigned int i = 0; i < n; i++)
			{
				double dot = DotProduct(refNormal, incNormals[i]);
				if (dot < minDot)
				{
					minDot = dot;
					incidentFace = i;
				}
			}

			v[0] = incVertices[incidentFace];
			incidentFace = incidentFace + 1 < n ? incidentFace + 1 : 0;
			v[1] = incVertices[incidentFace];
		}

		bool AABB_to_AABB(SPC_AABB& a, SPC_AABB& b)
		{
			if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
			if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
			return true;
		}

		bool AABB_to_AABB(SPC_Manifold* m)
		{
			SPC_AABB* a = reinterpret_cast<SPC_AABB*>(m->A);
			SPC_AABB* b = reinterpret_cast<SPC_AABB*>(m->B);
			m->contact_count = 0;

			if (!AABB_to_AABB(*a, *b)) return false;

			Vec2 n = b->GetCenter() - a->GetCenter();

			double a_extent = (a->max.x - a->min.x) / 2;
			double b_extent = (b->max.x - b->min.x) / 2;

			double x_overlap = a_extent + b_extent - std::abs(n.x);

			a_extent = (a->max.y - a->min.y) / 2;
			b_extent = (b->max.y - b->min.y) / 2;

			double y_overlap = a_extent + b_extent - std::abs(n.y);

			if (x_overlap < y_overlap)
			{
				if (n.x < 0)
					m->normal = Vec2(-1, 0);
				else
					m->normal = Vec2(1, 0);

				m->penetration = x_overlap;

				m->contact_count = 2;
				m->contact_points[0] = Vec2((n.x < 0 ? a->min.x : a->max.x), std::max(a->min.y, b->min.y));
				m->contact_points[1] = Vec2(m->contact_points[0].x, std::min(a->max.y, b->max.y));
			}
			else
			{
				if (n.y < 0)
					m->normal = Vec2(0, -1);
				else
					m->normal = Vec2(0, 1);

				m->penetration = y_overlap;

				m->contact_count = 2;
				m->contact_points[0] = Vec2(std::max(a->min.x, b->min.x), (n.y < 0 ? a->min.y : a->max.y));
				m->contact_points[1] = Vec2(std::min(a->max.x, b->max.x), m->contact_points[0].y);
			}

			return true;
		}

		bool AABB_to_Circle(SPC_AABB& a, SPC_Circle& b)
		{
			Vec2 n = b.position - (a.min + a.max) / 2;

			Vec2 closest = n; //get closest point from A to B
			double x_extent = (a.max.x - a.min.x) / 2, y_extent = (a.max.y - a.min.y) / 2;
			closest.x = Clamp(closest.x, -x_extent, x_extent);
			closest.y = Clamp(closest.y, -y_extent, y_extent);

			if (n == closest) //circle is inside AABB
				return true;

			Vec2 normal = n - closest;
			double d = normal.LengthSquared();
			double r = b.radius;

			if (d > r * r) return false;
			return true;
		}

		bool AABB_to_Circle(SPC_Manifold* m)
		{
			SPC_AABB* a = reinterpret_cast<SPC_AABB*>(m->A);
			SPC_Circle* b = reinterpret_cast<SPC_Circle*>(m->B);

			Vec2 n = b->position - a->GetCenter();

			Vec2 closest = n; //get closest point from A to B
			double x_extent = (a->max.x - a->min.x) / 2, y_extent = (a->max.y - a->min.y) / 2;
			closest.x = Clamp(closest.x, -x_extent, x_extent);
			closest.y = Clamp(closest.y, -y_extent, y_extent);

			bool inside = false;
			if (n == closest) //circle is inside AABB
			{
				inside = true;

				//find closest axis
				if (std::abs(n.x) > std::abs(n.y)) //x axis
					closest.x = closest.x > 0 ? x_extent : -x_extent;
				else
					closest.y = closest.y > 0 ? y_extent : -y_extent;
			}

			Vec2 normal = n - closest;
			double d = normal.LengthSquared();
			double r = b->radius;

			if (d > r * r && !inside) return false;

			d = std::sqrt(d);
			normal.Normalize();

			m->contact_count = 1;
			m->normal = inside ? -normal : normal;
			m->penetration = inside ? 2 * r : r - d;
			m->contact_points[0] = a->GetCenter() + closest;

			return true;
		}

		bool Circle_to_AABB(SPC_Manifold* m)
		{
			std::swap(m->A, m->B);
			bool ret = AABB_to_Circle(m);
			std::swap(m->A, m->B);
			m->normal = -m->normal;

			return ret;
		}

		bool AABB_to_Polygon(SPC_AABB& a, SPC_Polygon& b)
		{
			Vec2 a_vertice[4];
			a.GetVertices(a_vertice);

			unsigned int faceA;
			double penetrationA = FindAxisLeastPenetration(&faceA, a_vertice, (Vec2*)AABB_normals, 4, b.vertices, b.vertexCount);
			if (penetrationA >= 0)
				return false;

			unsigned int faceB;
			double penetrationB = FindAxisLeastPenetration(&faceB, b.vertices, b.normals, b.vertexCount, a_vertice, 4);
			if (penetrationB >= 0)
				return false;

			unsigned int referenceIndex, refSize, incSize;

			Vec2* refVertices, * refNormals;
			Vec2* incVertices, * incNormals;

			if (penetrationA >= penetrationB)
			{
				refVertices = a_vertice;
				refNormals = (Vec2*)AABB_normals;
				incVertices = b.vertices;
				incNormals = b.normals;

				refSize = 4;
				incSize = b.vertexCount;
				referenceIndex = faceA;
			}
			else
			{
				refVertices = b.vertices;
				refNormals = b.normals;
				incVertices = a_vertice;
				incNormals = (Vec2*)AABB_normals;

				refSize = b.vertexCount;
				incSize = 4;
				referenceIndex = faceB;
			}

			Vec2 incidentFace[2];
			FindIncidentFace(incidentFace, refNormals[referenceIndex], incVertices, incNormals, incSize);

			Vec2 v1 = refVertices[referenceIndex];
			referenceIndex = referenceIndex + 1 < refSize ? referenceIndex + 1 : 0;
			Vec2 v2 = refVertices[referenceIndex];

			Vec2 sidePlaneNormal = v2 - v1;
			sidePlaneNormal.Normalize();

			Vec2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);

			double negSide = -DotProduct(sidePlaneNormal, v1);
			double posSide = DotProduct(sidePlaneNormal, v2);

			if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
				return false;

			if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
				return false;

			for (int i = 0; i < 2; i++)
			{
				double separation = DotProduct(refFaceNormal, incidentFace[i]);
				if (separation <= 0) return true;
			}

			return false;
		}

		bool AABB_to_Polygon(SPC_Manifold* m)
		{
			SPC_AABB* a = reinterpret_cast<SPC_AABB*>(m->A);
			SPC_Polygon* b = reinterpret_cast<SPC_Polygon*>(m->B);

			Vec2 a_vertice[4];
			a->GetVertices(a_vertice);

			unsigned int faceA;
			double penetrationA = FindAxisLeastPenetration(&faceA, a_vertice, (Vec2*)AABB_normals, 4, b->vertices, b->vertexCount);
			if (penetrationA >= 0)
				return false;

			unsigned int faceB;
			double penetrationB = FindAxisLeastPenetration(&faceB, b->vertices, b->normals, b->vertexCount, a_vertice, 4);
			if (penetrationB >= 0)
				return false;

			unsigned int referenceIndex, refSize, incSize;

			Vec2* refVertices, * refNormals;
			Vec2* incVertices, * incNormals;

			bool flip;

			if (penetrationA >= penetrationB)
			{
				refVertices = a_vertice;
				refNormals = (Vec2*)AABB_normals;
				incVertices = b->vertices;
				incNormals = b->normals;

				refSize = 4;
				incSize = b->vertexCount;
				referenceIndex = faceA;
				flip = false;
			}
			else
			{
				refVertices = b->vertices;
				refNormals = b->normals;
				incVertices = a_vertice;
				incNormals = (Vec2*)AABB_normals;

				refSize = b->vertexCount;
				incSize = 4;
				referenceIndex = faceB;
				flip = true;
			}

			Vec2 incidentFace[2];
			FindIncidentFace(incidentFace, refNormals[referenceIndex], incVertices, incNormals, incSize);

			Vec2 v1 = refVertices[referenceIndex];
			referenceIndex = referenceIndex + 1 < refSize ? referenceIndex + 1 : 0;
			Vec2 v2 = refVertices[referenceIndex];

			Vec2 sidePlaneNormal = v2 - v1;
			sidePlaneNormal.Normalize();

			Vec2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);

			double refC = DotProduct(refFaceNormal, v1);
			double negSide = -DotProduct(sidePlaneNormal, v1);
			double posSide = DotProduct(sidePlaneNormal, v2);

			if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
				return false;

			if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
				return false;

			m->normal = flip ? -refFaceNormal : refFaceNormal;

			unsigned int cp = 0;
			double separation = DotProduct(refFaceNormal, incidentFace[0]) - refC;

			if (separation <= 0)
			{
				m->contact_points[cp] = incidentFace[0];
				m->penetration = -separation;
				cp++;
			}
			else
				m->penetration = 0;

			separation = DotProduct(refFaceNormal, incidentFace[1]) - refC;
			if (separation <= 0)
			{
				m->contact_points[cp] = incidentFace[1];
				m->penetration += -separation;
				cp++;

				m->penetration /= cp;
			}

			m->contact_count = cp;
			return m->contact_count > 0;
		}

		bool Polygon_to_AABB(SPC_Manifold* m)
		{
			std::swap(m->A, m->B);
			bool ret = AABB_to_Polygon(m);
			std::swap(m->A, m->B);
			m->normal = -m->normal;

			return ret;
		}

		bool Circle_to_Circle(SPC_Circle& a, SPC_Circle& b)
		{
			double r = a.radius + b.radius;
			Vec2 n = b.position - a.position;
			double d = n.LengthSquared();

			if (r * r < d) return false;
			return true;
		}

		bool Circle_to_Circle(SPC_Manifold* m)
		{
			SPC_Circle* a = reinterpret_cast<SPC_Circle*>(m->A);
			SPC_Circle* b = reinterpret_cast<SPC_Circle*>(m->B);

			Vec2 n = b->position - a->position;
			double r = a->radius + b->radius;
			double d = n.LengthSquared();
			m->contact_count = 0;

			if (d > r * r)
				return false;

			d = std::sqrt(d);
			m->contact_count = 1;

			if (d != 0)
			{
				m->penetration = r - d;
				m->normal = n / d;
				m->contact_points[0] = m->normal * a->radius + a->position;
			}
			else
			{
				m->penetration = a->radius;
				m->normal = { 1,0 };
				m->contact_points[0] = a->position;
			}
		}

		bool Circle_to_Polygon(SPC_Circle& a, SPC_Polygon& b)
		{
			double r = a.radius * a.radius;
			for (unsigned int i = 0; i < b.vertexCount; i++)
				if (r * r <= DistanceSquared(a.position, b.vertices[i])) return true;
			return false;
		}

		bool Circle_to_Polygon(SPC_Manifold* m)
		{
			SPC_Circle* a = reinterpret_cast<SPC_Circle*>(m->A);
			SPC_Polygon* b = reinterpret_cast<SPC_Polygon*>(m->B);

			m->contact_count = 0;

			Vec2 center = a->position;

			double separation = -DBL_MAX;
			unsigned int faceNormal = 0;

			for (unsigned int i = 0; i < b->vertexCount; i++)
			{
				double s = DotProduct(b->normals[i], center - b->vertices[i]);

				if (s > a->radius) return false;

				if (s > separation)
				{
					separation = s;
					faceNormal = i;
				}
			}

			Vec2 v1 = b->vertices[faceNormal];
			unsigned int i2 = faceNormal + 1 < b->vertexCount ? faceNormal + 1 : 0;
			Vec2 v2 = b->vertices[i2];

			if (separation < 1e-4)
			{
				m->contact_count = 1;
				m->normal = -b->normals[faceNormal];
				m->contact_points[0] = m->normal * a->radius + a->position;
				m->penetration = a->radius;
				return true;
			}

			double dot1 = DotProduct(center - v1, v2 - v1);
			double dot2 = DotProduct(center - v2, v1 - v2);
			m->penetration = a->radius - separation;

			if (dot1 <= 0)
			{
				if (DistanceSquared(center, v1) > a->radius * a->radius)
					return false;

				m->contact_count = 1;
				Vec2 n = v1 - center;
				n.Normalize();

				m->normal = n;
				m->contact_points[0] = v1;
			}
			else if (dot2 <= 0)
			{
				if (DistanceSquared(center, v2) > a->radius * a->radius)
					return false;

				m->contact_count = 1;
				Vec2 n = v2 - center;
				n.Normalize();

				m->normal = n;
				m->contact_points[0] = v2;
			}
			else
			{
				Vec2 n = b->normals[faceNormal];
				if (DotProduct(center - v1, n) > a->radius)
					return false;

				m->normal = -n;
				m->contact_points[0] = m->normal * a->radius + a->position;
				m->contact_count = 1;
			}

			return true;
		}

		bool Polygon_to_Circle(SPC_Manifold* m)
		{
			std::swap(m->A, m->B);
			bool ret = Circle_to_Polygon(m);
			std::swap(m->A, m->B);
			m->normal = -m->normal;

			return ret;
		}

		bool Polygon_to_Polygon(SPC_Polygon a, SPC_Polygon b)
		{
			unsigned int faceA;
			double penetrationA = FindAxisLeastPenetration(&faceA, a.vertices, a.normals, a.vertexCount, b.vertices, b.vertexCount);
			if (penetrationA >= 0)
				return false;

			unsigned int faceB;
			double penetrationB = FindAxisLeastPenetration(&faceB, b.vertices, b.normals, b.vertexCount, a.vertices, a.vertexCount);
			if (penetrationB >= 0)
				return false;

			unsigned int referenceIndex;

			SPC_Polygon* ref, * inc;

			if (penetrationA >= penetrationB)
			{
				ref = &a;
				inc = &b;
				referenceIndex = faceA;
			}
			else
			{
				ref = &b;
				inc = &a;
				referenceIndex = faceB;
			}

			Vec2 incidentFace[2];
			FindIncidentFace(incidentFace, ref->normals[referenceIndex], inc->vertices, inc->normals, inc->vertexCount);

			Vec2 v1 = ref->vertices[referenceIndex];
			referenceIndex = referenceIndex + 1 < ref->vertexCount ? referenceIndex + 1 : 0;
			Vec2 v2 = ref->vertices[referenceIndex];

			Vec2 sidePlaneNormal = v2 - v1;
			sidePlaneNormal.Normalize();

			Vec2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);

			double refC = DotProduct(refFaceNormal, v1);
			double negSide = -DotProduct(sidePlaneNormal, v1);
			double posSide = DotProduct(sidePlaneNormal, v2);

			if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
				return false;

			if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
				return false;

			for (int i = 0; i < 2; i++)
			{
				double separation = DotProduct(refFaceNormal, incidentFace[i]);
				if (separation <= 0) return true;
			}

			return false;
		}

		bool Polygon_to_Polygon(SPC_Manifold* m)
		{
			SPC_Polygon* a = reinterpret_cast<SPC_Polygon*>(m->A);
			SPC_Polygon* b = reinterpret_cast<SPC_Polygon*>(m->B);

			unsigned int faceA;
			double penetrationA = FindAxisLeastPenetration(&faceA, a->vertices, a->normals, a->vertexCount, b->vertices, b->vertexCount);
			if (penetrationA >= 0)
				return false;

			unsigned int faceB;
			double penetrationB = FindAxisLeastPenetration(&faceB, b->vertices, b->normals, b->vertexCount, a->vertices, a->vertexCount);
			if (penetrationB >= 0)
				return false;

			unsigned int referenceIndex;

			SPC_Polygon* ref, * inc;
			bool flip;

			if (penetrationA >= penetrationB)
			{
				ref = a;
				inc = b;
				referenceIndex = faceA;
				flip = false;
			}
			else
			{
				ref = b;
				inc = a;
				referenceIndex = faceB;
				flip = true;
			}

			Vec2 incidentFace[2];
			FindIncidentFace(incidentFace, ref->normals[referenceIndex], inc->vertices, inc->normals, inc->vertexCount);

			Vec2 v1 = ref->vertices[referenceIndex];
			referenceIndex = referenceIndex + 1 == ref->vertexCount ? 0 : referenceIndex + 1;
			Vec2 v2 = ref->vertices[referenceIndex];

			Vec2 sidePlaneNormal = v2 - v1;
			sidePlaneNormal.Normalize();

			Vec2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);

			double refC = DotProduct(refFaceNormal, v1);
			double negSide = -DotProduct(sidePlaneNormal, v1);
			double posSide = DotProduct(sidePlaneNormal, v2);

			if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
				return false;

			if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
				return false;

			m->normal = flip ? -refFaceNormal : refFaceNormal;

			unsigned int cp = 0;
			double separation = DotProduct(refFaceNormal, incidentFace[0]) - refC;

			if (separation <= 0)
			{
				m->contact_points[cp] = incidentFace[0];
				m->penetration = -separation;
				cp++;
			}
			else
				m->penetration = 0;

			separation = DotProduct(refFaceNormal, incidentFace[1]) - refC;
			if (separation <= 0)
			{
				m->contact_points[cp] = incidentFace[1];
				m->penetration += -separation;
				cp++;

				m->penetration /= cp;
			}

			m->contact_count = cp;
			return m->contact_count > 0;
		}

		bool Collide(SPC_Manifold* m)
		{
			return CollideFunc[m->A->type][m->B->type](m);
		}
	}
}