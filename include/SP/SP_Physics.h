#pragma once
#ifndef __SP_PHYSICS__
#define __SP_PHYSICS__

#include <algorithm>
#include <functional>

#include "SP_Shapes.h"
#include "SP_Body.h"
#include "SP_Manifold.h"
#include "SP_Math.h"
#include "SP_Mat22.h"

namespace SP2
{
	bool Intersect_Circle_to_Circle(SP_Manifold* manifold)
	{
		SP_Circle* a = reinterpret_cast<SP_Circle*>(manifold->A->shape);
		SP_Circle* b = reinterpret_cast<SP_Circle*>(manifold->B->shape);

		double r = a->radius + b->radius;
		Vec2 n = manifold->B->position - manifold->A->position;
		double d = n.LengthSquared();

		if (r * r < d)
		{
			manifold->contact_count = 0;
			return false;
		}

		d = std::sqrt(d);
		manifold->contact_count = 1;

		if (d != 0)
		{
			manifold->penetration = r - d;
			manifold->normal = n / d;
			manifold->contacts[0] = manifold->normal * a->radius + manifold->A->position;
		}
		else
		{
			manifold->penetration = a->radius;
			manifold->normal = { 1,0 };
			manifold->contacts[0] = manifold->A->position;
		}

		return true;
	}

	bool Intersect_Circle_to_Polygon(SP_Manifold* manifold)
	{
		SP_Circle* a = reinterpret_cast<SP_Circle*>(manifold->A->shape);
		SP_Polygon* b = reinterpret_cast<SP_Polygon*>(manifold->B->shape);

		manifold->contact_count = 0;
		
		Vec2 center = manifold->A->position;
		SP_Mat22 u = SP_Mat22(manifold->B->orientation + b->orientation);
		center = u.Transpose() * (center - manifold->B->position);

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
			manifold->contact_count = 1;
			manifold->normal = -(u * b->normals[faceNormal]);
			manifold->contacts[0] = manifold->normal * a->radius + manifold->A->position;
			manifold->penetration = a->radius;
			return true;
		}

		double dot1 = DotProduct(center - v1, v2 - v1);
		double dot2 = DotProduct(center - v2, v1 - v2);
		manifold->penetration = a->radius - separation;

		if (dot1 <= 0)
		{
			if (DistanceSquared(center, v1) > a->radius * a->radius)
				return false;

			manifold->contact_count = 1;
			Vec2 n = v1 - center;
			n = u * n;
			n.Normalize();

			manifold->normal = n;
			v1 = u * v1 + manifold->B->position;
			manifold->contacts[0] = v1;
		}
		else if (dot2 <= 0)
		{
			if (DistanceSquared(center, v2) > a->radius * a->radius)
				return false;

			manifold->contact_count = 1;
			Vec2 n = v2 - center;
			v2 = u * v2 + manifold->B->position;
			manifold->contacts[0] = v2;

			n = u * n;
			n.Normalize();
			manifold->normal = n;
		}
		else
		{
			Vec2 n = b->normals[faceNormal];
			if (DotProduct(center - v1, n) > a->radius)
				return false;

			n = u * n;
			manifold->normal = -n;
			manifold->contacts[0] = manifold->normal * a->radius + manifold->A->position;
			manifold->contact_count = 1;
		}

		return true;
	}

	bool Intersect_Polygon_to_Circle(SP_Manifold* manifold)
	{
		std::swap(manifold->A, manifold->B);
		bool ret = Intersect_Circle_to_Polygon(manifold);
		manifold->normal = -manifold->normal;
		std::swap(manifold->A, manifold->B);
		return ret;
	}

	double FindAxisLeastPenetration(unsigned int* faceIndex, SP_Body* a, SP_Body* b)
	{
		SP_Polygon* A = reinterpret_cast<SP_Polygon*>(a->shape);
		SP_Polygon* B = reinterpret_cast<SP_Polygon*>(b->shape);

		double bestDistance = -DBL_MAX;
		unsigned int bestIndex = 0;

		SP_Mat22 Au = SP_Mat22(a->orientation);
		SP_Mat22 buT = SP_Mat22(b->orientation + B->orientation).Transpose();

		for (unsigned int i = 0; i < A->vertexCount; i++)
		{
			Vec2 n = A->normals[i];
			Vec2 nw = Au * n;

			n = buT * nw;

			Vec2 s = B->GetSupport(-n);

			Vec2 v = A->vertices[i];
			v = Au * v + a->position;
			v -= b->position;
			v = buT * v;

			double d = DotProduct(n, s - v);

			if (d > bestDistance)
			{
				bestDistance = d;
				bestIndex = i;
			}
		}

		*faceIndex = bestIndex;
		return bestDistance;
	}

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

	void FindIncidentFace(Vec2* v, SP_Body* refBody, SP_Body* incBody, unsigned int referenceIndex)
	{
		SP_Polygon* ref = reinterpret_cast<SP_Polygon*>(refBody->shape);
		SP_Polygon* inc = reinterpret_cast<SP_Polygon*>(incBody->shape);

		Vec2 refNormal = ref->normals[referenceIndex];
		SP_Mat22 u_ref(refBody->orientation + ref->orientation);
		SP_Mat22 u_inc(incBody->orientation + inc->orientation);

		refNormal = u_ref * refNormal;
		refNormal = u_inc.Transpose() * refNormal;

		int incidentFace = 0;
		double minDot = DBL_MAX;
		for (unsigned int i = 0; i < inc->vertexCount; i++)
		{
			double dot = DotProduct(refNormal, inc->normals[i]);
			if (dot < minDot)
			{
				minDot = dot;
				incidentFace = i;
			}
		}

		v[0] = u_inc * inc->vertices[incidentFace] + incBody->position;
		incidentFace = incidentFace + 1 < inc->vertexCount ? incidentFace + 1 : 0;
		v[1] = u_inc * inc->vertices[incidentFace] + incBody->position;
	}

	bool Intersect_Polygon_to_Polygon(SP_Manifold* manifold)
	{
		SP_Polygon* a = reinterpret_cast<SP_Polygon*>(manifold->A->shape);
		SP_Polygon* b = reinterpret_cast<SP_Polygon*>(manifold->B->shape);

		unsigned int faceA;
		double penetrationA = FindAxisLeastPenetration(&faceA, manifold->A, manifold->B);
		if (penetrationA >= 0)
			return false;

		unsigned int faceB;
		double penetrationB = FindAxisLeastPenetration(&faceB, manifold->B, manifold->A);
		if (penetrationB >= 0)
			return false;

		unsigned int referenceIndex;

		SP_Body* refBody, *incBody;
		SP_Polygon* ref, *inc;
		bool flip;

		if (penetrationA >= penetrationB)
		{
			ref = a;
			refBody = manifold->A;
			inc = b;
			incBody = manifold->B;
			referenceIndex = faceA;
			flip = false;
		}
		else
		{
			ref = b;
			refBody = manifold->B;
			inc = a;
			incBody = manifold->A;
			referenceIndex = faceB;
			flip = true;
		}

		Vec2 incidentFace[2];
		FindIncidentFace(incidentFace, refBody, incBody, referenceIndex);

		Vec2 v1 = ref->vertices[referenceIndex];
		referenceIndex = referenceIndex + 1 == ref->vertexCount ? 0 : referenceIndex + 1;
		Vec2 v2 = ref->vertices[referenceIndex];

		SP_Mat22 u_ref(refBody->orientation + ref->orientation);

		v1 = u_ref * v1 + refBody->position;
		v2 = u_ref * v2 + refBody->position;

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

		manifold->normal = flip ? -refFaceNormal : refFaceNormal;

		unsigned int cp = 0;
		double separation = DotProduct(refFaceNormal, incidentFace[0]) - refC;

		if (separation <= 0)
		{
			manifold->contacts[cp] = incidentFace[0];
			manifold->penetration = -separation;
			cp++;
		}
		else
			manifold->penetration = 0;

		separation = DotProduct(refFaceNormal, incidentFace[1]) - refC;
		if (separation <= 0)
		{
			manifold->contacts[cp] = incidentFace[1];
			manifold->penetration += -separation;
			cp++;

			manifold->penetration /= cp;
		}

		manifold->contact_count = cp;
		return manifold->contact_count > 0;
	}

	void CorrectPosition(SP_Manifold* manifold)
	{
		const double percent = 0.4; //0.2 ~ 0.8. percent of normal
		const double slop = 0.05; //0.01 ~ 0.1. prevention threshold of jitter front and back.

		Vec2 correction = (std::max(manifold->penetration - slop, 0.0) / (manifold->A->massData.inv_mass + manifold->B->massData.inv_mass)) * percent * manifold->normal;
		manifold->A->position -= manifold->A->massData.inv_mass * correction;
		manifold->B->position += manifold->B->massData.inv_mass * correction;
	}

	void ResolveCollision(SP_Manifold* manifold)
	{
		Vec2 rv = manifold->A->velocity - manifold->B->velocity; //relative velocity
		double nv = DotProduct(rv, manifold->normal); //normal velocity

		if (nv > 0) return;
		double e = std::min(manifold->A->material.restitution, manifold->B->material.restitution);

		double scalar = -(e + 1) * nv;
		scalar /= manifold->A->massData.inv_mass + manifold->B->massData.inv_mass;

		Vec2 impulse = scalar * manifold->normal;
		manifold->A->velocity -= manifold->A->massData.inv_mass * impulse;
		manifold->B->velocity += manifold->B->massData.inv_mass * impulse;

		//friction
		Vec2 tangent = rv - nv * manifold->normal;
		tangent.Normalize();

		double scalar_t = -DotProduct(rv, tangent);
		scalar_t /= (manifold->A->massData.inv_mass + manifold->B->massData.inv_mass);

		double mu = Pythagorean(manifold->A->material.friction_static, manifold->B->material.friction_static);
		Vec2 frictionImpulse;

		if (std::abs(scalar_t) < scalar * mu)
			frictionImpulse = scalar_t * tangent;
		else
		{
			double friction_dynamic = Pythagorean(manifold->A->material.friction_dynamic, manifold->B->material.friction_dynamic);
			frictionImpulse = -scalar * tangent * friction_dynamic;
		}

		manifold->A->velocity -= manifold->A->massData.inv_mass * frictionImpulse;
		manifold->B->velocity += manifold->B->massData.inv_mass * frictionImpulse;
	}

	typedef bool (*CollideCallback) (SP_Manifold* manifold);

	CollideCallback CollideTest[SP_Shape::Count][SP_Shape::Count] =
	{
		{ Intersect_Circle_to_Circle, Intersect_Circle_to_Polygon },
		{ Intersect_Polygon_to_Circle, Intersect_Polygon_to_Polygon }
	};
}
#endif