
#include "PrimitiveQuery.h"
#include "HullCollider.h"
#include "Body.h"
#include "SphereCollider.h"
#include "BroadPhase.h"
/// <summary>
/// Whether point in the collider(OK)
/// </summary>
/// <param name="collider"></param>
/// <param name="point"></param>
/// <returns></returns>
bool QueryPoint(Collider* collider, const glm::vec3& point)
{
	switch (collider->GetShape())
	{
	case (Collider::Hull):
	{
		HullCollider* c = static_cast<HullCollider*>(collider);
		for (int i = 0; i < c->GetFaceCount(); i++)
		{
			glm::vec3 P = c->GetFace(i)->edge->tail->position;
			P = c->GetBody()->LocalToGlobalPoint(P);
			glm::vec3 n = c->GetFace(i)->normal;
			n = c->GetBody()->LocalToGlobalVec(n);
			if (glm::dot(P - point, n) < -1e-4f)
			{
				//the point must be outside the face
				return false;
			}
		}
		return true;
	}
	case (Collider::Sphere):
	{
		SphereCollider* c = static_cast<SphereCollider*>(collider);
		glm::vec3 C = c->getCenterWorldPosition();
		return (glm::l2Norm(point - C) < c->GetRadius());
	}
	default:
		return false;
	}
}

void QueryPoint(std::vector<ParticleContact>& contacts, Collider* collider, Particle* particle)
{
	switch (collider->GetShape())
	{
	case (Collider::Hull):
	{
		glm::vec3 n(0);
		float d(0);
		HullCollider* c = static_cast<HullCollider*>(collider);
		for (int i = 0; i < c->GetFaceCount(); i++)
		{
			glm::vec3 P = c->GetFace(i)->edge->tail->position;
			P = c->GetBody()->LocalToGlobalPoint(P);
			n = c->GetFace(i)->normal;
			n = c->GetBody()->LocalToGlobalVec(n);
			d = glm::dot(P - particle->GetPosition(), n);
			if (d < -0.07f)
				return;
		}
		contacts.push_back(ParticleContact(particle, n, -d));
		break;
	}
	case (Collider::Sphere):
	{
		SphereCollider* c = static_cast<SphereCollider*>(collider);
		glm::vec3 C = c->GetBody()->LocalToGlobalPoint(c->GetCentroid());
		glm::vec3 n = particle->GetPosition() - C;
		float d = glm::length2(n);
		if (d < c->GetRadius() * c->GetRadius() * 1.1f)
		{
			d = std::sqrtf(d);
			contacts.push_back(ParticleContact(particle, n / d, c->GetRadius() - d));
		}
		break;
	}
	default:
		assert(false);
		break;
	}
}

bool IntersectSegmentSphere(const glm::vec3& A, const glm::vec3& B, SphereCollider* s, glm::vec3& P)
{
	glm::vec3 m = A - s->GetBody()->GetPosition();
	float l = glm::length(B - A);
	glm::vec3 d = (B - A) / l;
	float b = glm::dot(m, d);
	float c = glm::dot(m, m) - s->GetRadius() * s->GetRadius();

	// A is outside, and AB is pointing away
	if (c > 0.0f && b > 0.0f)
		return false;

	float disc = b * b - c;

	if (disc < 0.0f)
		return false;

	float t = -b - std::sqrtf(disc);

	if (t > l)
		return false;

	t = t < 0 ? 0 : -b;
	P = A + t * d;

	return true;
}

bool TriangleIsCCW(const glm::vec3& A, const glm::vec3& B, const glm::vec3& C, const glm::vec3& normal)
{
	return (glm::dot(glm::cross(B - A, C - B), normal) >= 0);
}

bool QueryPoint(const glm::vec3& P, const std::vector<glm::vec3>& verts, const glm::vec3& normal)
{
	int n = verts.size();

	int low = 0, high = n, mid = 0;
	do {
		mid = (low + high) / 2;
		if (TriangleIsCCW(verts[0], verts[mid], P, normal))
			low = mid;
		else
			high = mid;

	} while (low + 1 < high);

	if (low == 0 || high == n)
		return false;

	return (TriangleIsCCW(verts[low], verts[high], P, normal));
}

bool Overlap(AABB* A, AABB* B)
{
	if (A->max.x < B->min.x || A->min.x > B->max.x ||
		A->max.y < B->min.y || A->min.y > B->max.y ||
		A->max.z < B->min.z || A->min.z > B->max.z)
	{
		return false;
	}
	return true;
}