
#include "NarrowPhase.h"
#include "Collider.h"
#include "HullCollider.h"
#include "Contact.h"
#include "Body.h"
#include "Geometry.h"
#include "PrimitiveQuery.h"

#define EPSILON 0.005

bool QueryFaceAxes(FaceQuery& query, HullCollider* A, HullCollider* B)
{
	glm::vec3 axis, vA;
	float separation = 0.0f;
	float maxSeparation = -FLT_MAX;
	int support, bestFace = -1, bestVertex = -1;

	for (int i = 0; i < A->GetFaceCount(); i++)
	{
		axis = A->GetFace(i)->normal;
		axis = B->GetBody()->LocalToLocalVec(A->GetBody(), axis);

		support = B->GetSupport(-axis);				// max projection of B on axis

		vA = A->GetFace(i)->edge->tail->position;			// min prohection of A on axis
		vA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), vA);

		separation = glm::dot(B->GetVertex(support)->position - vA, axis);	// compare projections

		if (separation > 0.0f)						// early out?
			return false;

		if (separation > maxSeparation)
		{
			maxSeparation = separation;
			bestFace = i;
			bestVertex = support;
		}
	}
	query.faceIndex = bestFace;
	query.separation = maxSeparation;
	query.vertIndex = bestVertex;

	return true;
}

bool QueryEdgeAxes(EdgeQuery& query, HullCollider* A, HullCollider* B)
{
	glm::vec3 edgeA, pA, f1A, f2A;
	glm::vec3 edgeB, pB, f1B, f2B;
	float separation = 0.0f;
	float maxSeparation = -FLT_MAX;
	int idA = -1, idB = -1;	// minimum penetrating edges
	glm::vec3 bestAxis;		// minimum penetration axis

	// centroid of A in B's space
	glm::vec3 cA = A->GetCentroid();
	cA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), cA);

	for (int iA = 0; iA < A->GetEdgeCount(); iA++)
	{
		edgeA = A->GetEdge(iA)->GetDirection();
		edgeA = B->GetBody()->LocalToLocalVec(A->GetBody(), edgeA);
		f1A = A->GetEdge(iA)->face->normal;
		f1A = B->GetBody()->LocalToLocalVec(A->GetBody(), f1A);
		f2A = A->GetEdge(iA)->twin->face->normal;
		f2A = B->GetBody()->LocalToLocalVec(A->GetBody(), f2A);

		for (int iB = 0; iB < B->GetEdgeCount(); iB++)
		{
			edgeB = B->GetEdge(iB)->GetDirection();
			f1B = B->GetEdge(iB)->face->normal;
			f2B = B->GetEdge(iB)->twin->face->normal;

			if (IsMinkowskiFace(f1A, f2A, -edgeA, -f1B, -f2B, -edgeB))
			{
				glm::vec3 axis = glm::cross(edgeA, edgeB);
				float L = glm::length(axis);

				glm::vec3 v(10, 0, 0);
				float len = glm::length(v);
				v = glm::normalize(v);
				glm::vec3 vn = v / len;

				// skip near parallel edges
				const float tolerance = 0.005f;
				if (L < tolerance * glm::sqrt(glm::length2(edgeA) * glm::length2(edgeB)))
					continue;

				pA = A->GetEdge(iA)->tail->position;
				pA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), pA);
				// ensure consistent normal orientation (from A to B)
				if (glm::dot(pA - cA, axis) < 0.0f)
					axis = -axis;

				axis /= L;

				pB = B->GetEdge(iB)->tail->position;
				separation = glm::dot(pB - pA, axis);

				if (separation > 0.0f)	return false;

				if (separation > maxSeparation)
				{
					maxSeparation = separation;
					idA = iA;
					idB = iB;
					bestAxis = axis;
				}
			}
		}
	}
	query.edgeIndex1 = idA;
	query.edgeIndex2 = idB;
	query.normal = B->GetBody()->LocalToGlobalVec(bestAxis);
	query.separation = maxSeparation;

	return true;
}

bool IsMinkowskiFace(const glm::vec3& A, const glm::vec3& B, const glm::vec3& B_x_A, const glm::vec3& C, const glm::vec3& D, const glm::vec3& D_x_C)
{
	// test if the arcs AB and CD intersect on the unit sphere
	float CBA = glm::dot(C, B_x_A);
	float DBA = glm::dot(D, B_x_A);
	float ADC = glm::dot(A, D_x_C);
	float BDC = glm::dot(B, D_x_C);

	return ((CBA * DBA < 0.0f) &&	// intersection test
		(ADC * BDC < 0.0f) &&	// intersection test
		(CBA * BDC > 0.0f));	// hemisphere test
}

void CreateEdgeContact(std::vector<Manifold>& manifolds, HullCollider* A, HullCollider* B, const EdgeQuery& query)
{
	HEdge* edgeA = A->GetEdge(query.edgeIndex1);
	HEdge* edgeB = B->GetEdge(query.edgeIndex2);
	glm::vec3 pA = A->GetBody()->LocalToGlobalPoint(edgeA->tail->position);
	glm::vec3 pB = B->GetBody()->LocalToGlobalPoint(edgeB->tail->position);
	glm::vec3 d1 = A->GetBody()->LocalToGlobalVec(edgeA->GetDirection());
	glm::vec3 d2 = B->GetBody()->LocalToGlobalVec(edgeB->GetDirection());
	glm::vec3 r = pA - pB;

	// Parameters to solve segment-segment overlap
	// To do: can be converted to 2D segment overlap by
	// projecting edgeB's end points on to the plane
	// passing through edgeA, and normal to both edges
	float a, b, c, d, e, f, s, t;
	a = glm::dot(d1, d1);
	b = glm::dot(d1, d2);
	c = glm::dot(d1, r);
	e = glm::dot(d2, d2);
	f = glm::dot(d2, r);

	d = a * e - b * b;
	//assert(d > 0.0f);
	//if (d <= 0)
		//return;

	s = (b * f - c * e) / d;
	//assert((s > 0.0f) && (s < 1.0f));
	// ToDo : This should never happen since SAT already reports overlapping edges
	// Edge-Edge collisions are currently buggy because of this
	// Possible causes are errors in face merging (not projecting faces to Newell planes or topological errors)
	// or non-sensical hull data (probably from splitting or incorrect deep copy)
	s = glm::clamp(s, 0.0f, 1.0f);

	t = (b * s + f) / e;

	if (t < 0.0f)
	{
		t = 0.0f;
		s = glm::clamp(-c / a, 0.0f, 1.0f);
	}
	else if (t > 1.0f)
	{
		t = 1.0f;
		s = glm::clamp((b - c) / a, 0.0f, 1.0f);
	}

	glm::vec3 pos = (pA + s * d1 + pB + t * d2) * 0.5f;

	Contact contact(A->GetBody(), B->GetBody(), pos, query.normal, -query.separation);
	Manifold m;
	m.contacts.push_back(contact);
	manifolds.push_back(m);
}

void CreateFaceContact(std::vector<Manifold>& manifolds, HullCollider* incident, HullCollider* reference, int incidentFace, int referenceFace)
{
	std::vector<glm::vec3> inPoly;			// the face to be clipped - vertices on reference face
	std::vector<HalfSpace> planes;	// the clipping planes - faces represented by each edge on the incident face

	// extract the vertices from incident face - in world space
	auto start = incident->GetFace(incidentFace)->edge;
	auto edge = start;
	do {
		inPoly.push_back(incident->GetBody()->LocalToGlobalPoint(edge->tail->position));
		edge = edge->next;
	} while (edge != start);

	{
		// extract the faces representing the clipping planes - in world space
		glm::vec3 normal, point;
		start = reference->GetFace(referenceFace)->edge;
		edge = start;
		do {
			normal = reference->GetBody()->LocalToGlobalVec(edge->twin->face->normal);
			point = reference->GetBody()->LocalToGlobalPoint(edge->tail->position);
			planes.push_back(HalfSpace(normal, point));
			edge = edge->next;
		} while (edge != start);
	}

	std::vector<glm::vec3> outPoly;		// the clipped poly
	glm::vec3 A, B;
	float dA, dB;

	for (auto plane : planes)
	{
		A = inPoly[inPoly.size() - 1];
		for (int i = 0; i < inPoly.size(); i++)
		{
			B = inPoly[i];
			dA = glm::dot(A, plane.normal) - plane.distance;
			dB = glm::dot(B, plane.normal) - plane.distance;

			// A and B Behind or On plane - out B
			if ((dA < 0.0f && dB < 0.0f) || (glm::abs(dA) < EPSILON) || (glm::abs(dB) < EPSILON))
			{
				outPoly.push_back(B);
			}
			// A Infront and B Behind - out I, B
			else if (dA > 0.0f && dB < 0.0f)
			{
				outPoly.push_back(A + (dA / (dA - dB)) * (B - A));
				outPoly.push_back(B);
			}
			// A Behind and B Infront - out I
			else if (dA < 0.0f && dB > 0.0f)
			{
				outPoly.push_back(A + (dA / (dA - dB)) * (B - A));
			}
			A = B;
		}

		if (outPoly.size() == 0)	// why is this happening?
		{
			return;
		}

		inPoly = outPoly;
		outPoly.clear();
	}

	// To Do : reduce clipped vertices size to <= 4

	glm::vec3 normal = reference->GetFace(referenceFace)->normal;
	normal = reference->GetBody()->LocalToGlobalVec(normal);
	glm::vec3 point = reference->GetFace(referenceFace)->edge->tail->position;
	point = reference->GetBody()->LocalToGlobalPoint(point);
	HalfSpace refFace(normal, point);
	float distance;
	int count = 0;
	Manifold m;

	for (glm::vec3 v : inPoly)
	{
		distance = refFace.Distance(v);
		if (distance <= EPSILON)
		{
			Contact contact(reference->GetBody(), incident->GetBody(), v, normal, -distance);
			m.contacts.push_back(contact);
		}
	}

	if (m.contacts.size() > 0)
		manifolds.push_back(m);
}

int FindIncidentFace(HullCollider* incident, HullCollider* reference, int referenceFace)
{
	int incidentFace;
	float dot;
	float minDot = FLT_MAX;
	glm::vec3 refNormal = reference->GetFace(referenceFace)->normal;
	refNormal = incident->GetBody()->LocalToLocalVec(reference->GetBody(), refNormal);

	for (int i = 0; i < incident->GetFaceCount(); i++)
	{
		dot = glm::dot(incident->GetFace(i)->normal, refNormal);
		if (dot < minDot)
		{
			minDot = dot;
			incidentFace = i;
		}
	}
	return incidentFace;
}

void DetectHullvsHull(std::vector<Manifold>& manifolds, HullCollider* A, HullCollider* B)
{
	FaceQuery faceQueryA;
	if (!QueryFaceAxes(faceQueryA, A, B))
		return;
	assert(faceQueryA.separation <= 0.0f);

	FaceQuery faceQueryB;
	if (!QueryFaceAxes(faceQueryB, B, A))
		return;
	assert(faceQueryB.separation <= 0.0f);

	EdgeQuery edgeQuery;
	if (!QueryEdgeAxes(edgeQuery, A, B))
		return;
	assert(edgeQuery.separation <= 0.0f);

	float maxFaceSep = faceQueryA.separation > faceQueryB.separation ? faceQueryA.separation : faceQueryB.separation;
	float epsilon = 0.05f;

	if (edgeQuery.separation > maxFaceSep + epsilon)
	{
		// edge contact
		CreateEdgeContact(manifolds, A, B, edgeQuery);
	}
	else
	{
		// face contact
		int incidentFace;
		if (faceQueryA.separation > faceQueryB.separation + epsilon)
		{
			incidentFace = FindIncidentFace(B, A, faceQueryA.faceIndex);
			CreateFaceContact(manifolds, B, A, incidentFace, faceQueryA.faceIndex);
		}
		else
		{
			incidentFace = FindIncidentFace(A, B, faceQueryB.faceIndex);
			CreateFaceContact(manifolds, A, B, incidentFace, faceQueryB.faceIndex);
		}
	}
}
//OK
void DetectSphereWithSphere(std::vector<Manifold>& manifolds, SphereCollider* A, SphereCollider* B)
{
	glm::vec3 CA = A->GetBody()->LocalToGlobalPoint(A->GetCentroid());
	glm::vec3 CB = B->GetBody()->LocalToGlobalPoint(B->GetCentroid());

	glm::vec3 normal = CB - CA;
	float dist2 = glm::l2Norm(normal);
	float ra = A->GetRadius();
	float rb = B->GetRadius();
	float rSum = ra + rb;

	if (dist2 > rSum)
		return;

	normal = glm::normalize(normal);


	glm::vec3 PA = CA + A->GetRadius() * normal;
	glm::vec3 PB = CB - B->GetRadius() * normal;
	glm::vec3 C = (PA + PB) * 0.5f;
	float penetration = rSum - dist2;

	Manifold m;
	m.contacts.push_back(std::move(Contact(A->GetBody(), B->GetBody(), C, normal, penetration)));
	manifolds.push_back(m);
}
/// <summary>
/// Check whether A sphere is collided with a Hull(OK)
/// </summary>
/// <param name="manifolds"></param>
/// <param name="A"></param>
/// <param name="B"></param>
void DetectSphereWithHull(std::vector<Manifold>& manifolds, SphereCollider* A, HullCollider* B)
{
	glm::vec3 center = A->GetBody()->GetPosition();	// sphere center

	if (QueryPoint(B, center))
	{
		float maxSeparation = -FLT_MAX;
		float separation = 0.0;
		int bestFitFace = -1;	// should be max separation

		for (int i = 0; i < B->GetFaceCount(); i++)
		{
			glm::vec3 normal = B->GetBody()->LocalToGlobalVec(B->GetFace(i)->normal);
			glm::vec3 origin = B->GetBody()->LocalToGlobalPoint(B->GetFace(i)->edge->tail->position);

			separation = glm::dot(origin - center, normal);
			if (separation > maxSeparation)
			{
				maxSeparation = separation;
				bestFitFace = i;
			}
		}

		glm::vec3 normal = B->GetBody()->LocalToGlobalVec(B->GetFace(bestFitFace)->normal);
		glm::vec3 contact = center - maxSeparation * normal;

		Manifold m;
		m.contacts.push_back(std::move(Contact(A->GetBody(), B->GetBody(), contact, -normal, -maxSeparation)));
		manifolds.push_back(m);
	}
	else
	{
		glm::vec3 normal(0);
		glm::vec3 point(0);
		HalfSpace plane;
		float distance = 0.0f;
		float minDistance = FLT_MAX;
		int bestFace = -1;

		for (int i = 0; i < B->GetFaceCount(); i++)
		{
			normal = B->GetBody()->LocalToGlobalVec(B->GetFace(i)->normal);
			point = B->GetBody()->LocalToGlobalPoint(B->GetFace(i)->edge->tail->position);
			plane = HalfSpace(normal, point);
			distance = plane.Distance(center);

			if (distance > 0.0f && distance < minDistance)
			{
				minDistance = distance;
				bestFace = i;
			}
		}

		if (minDistance > A->GetRadius())
			return;

		// intersect with closest face plane
		glm::vec3 contact;
		float penetration;
		normal = B->GetBody()->LocalToGlobalVec(B->GetFace(bestFace)->normal);
		contact = center - minDistance * normal;

		std::vector<glm::vec3> verts;
		HEdge* e = B->GetFace(bestFace)->edge;
		do {
			verts.push_back(B->GetBody()->LocalToGlobalPoint(e->tail->position));
			e = e->next;
		} while (e != B->GetFace(bestFace)->edge);

		if (QueryPoint(contact, verts, normal))
		{
			penetration = A->GetRadius() - minDistance;
			Manifold m;
			Contact c(A->GetBody(), B->GetBody(), contact, -normal, penetration);
			m.contacts.push_back(c);
			manifolds.push_back(m);
			return;
		}
		return;
		// intersect with edges of closest face
		e = B->GetFace(bestFace)->edge;
		glm::vec3 pA, pB;
		do {
			pA = B->GetBody()->LocalToGlobalPoint(e->tail->position);
			pB = B->GetBody()->LocalToGlobalPoint(e->twin->tail->position);
			if (IntersectSegmentSphere(pA, pB, A, contact))
			{
				normal = contact - center;
				float l = glm::length(normal);
				normal /= l;
				penetration = A->GetRadius() - l;
				Manifold m;
				Contact c(A->GetBody(), B->GetBody(), contact, normal, penetration);
				m.contacts.push_back(c);
				manifolds.push_back(m);
				return;
			}
			e = e->next;
		} while (e != B->GetFace(bestFace)->edge);
	}
}
//OK
void DetectCollision(std::vector<Manifold>& manifolds, Collider* A, Collider* B)
{
	if (A->GetShape() == Collider::Sphere && B->GetShape() == Collider::Sphere)
	{
		DetectSphereWithSphere(manifolds, static_cast<SphereCollider*>(A), static_cast<SphereCollider*>(B));
	}
	else if (A->GetShape() == Collider::Hull && B->GetShape() == Collider::Hull)
	{
		DetectHullvsHull(manifolds, static_cast<HullCollider*>(A), static_cast<HullCollider*>(B));
	}
	else if (A->GetShape() == Collider::Hull && B->GetShape() == Collider::Sphere)
	{
		DetectSphereWithHull(manifolds, static_cast<SphereCollider*>(B), static_cast<HullCollider*>(A));
	}
	else if (A->GetShape() == Collider::Sphere && B->GetShape() == Collider::Hull)
	{
		DetectSphereWithHull(manifolds, static_cast<SphereCollider*>(A), static_cast<HullCollider*>(B));
	}
	else {
		assert(false);
	}
}

glm::vec3 Simplex::FindClosestPoint() const
{
	switch (nVerts)
	{
	case 1:
		return vertices[0].point;
	case 2:
		return vertices[0].weight * vertices[0].point + vertices[1].weight * vertices[1].point;
	case 3:
		return vertices[0].weight * vertices[0].point + vertices[1].weight * vertices[1].point + vertices[2].weight * vertices[2].point;
	case 4:
		return glm::vec3(0, 0, 0);
	}
}

glm::vec3 Simplex::FindSearchDirection() const
{
	switch (nVerts)
	{
	case 1:
		return -vertices[0].point;
	case 2:
	{
		glm::vec3 AB = vertices[1].point - vertices[0].point;
		glm::vec3 AO = -vertices[0].point;
		glm::vec3 n = glm::cross(AB, AO);
		glm::vec3 dir = glm::cross(AB, n);

		if (glm::dot(AO, dir) < 0)
			dir *= -1.0f;

		return dir;
	}
	case 3:
	{
		glm::vec3 A = vertices[0].point;
		glm::vec3 B = vertices[1].point;
		glm::vec3 C = vertices[2].point;
		glm::vec3 AB = B - A;
		glm::vec3 AC = C - A;
		glm::vec3 n = glm::cross(AB, AC);
		glm::vec3 AO = -A;

		if (glm::dot(AO, n) < 0)
			n *= -1.0f;

		return n;
	}
	default:
		return glm::vec3(0, 0, 0);
	}
}

// Voronoi regions A, B, AB
void Simplex::Solve2()
{
	glm::vec3 A = vertices[0].point;
	glm::vec3 B = vertices[1].point;
	glm::vec3 AB = B - A;

	float u = glm::dot(B, AB);
	float v = glm::dot(-A, AB);

	if (v <= 0) // R A
	{
		vertices[0].weight = 1.0f;
		nVerts = 1;
	}
	else if (u <= 0) // R B
	{
		vertices[0] = vertices[1];
		vertices[0].weight = 1.0f;
		nVerts = 1;
	}
	else // R AB
	{
		//float invDenom = 1.0f / glm::dot(AB, AB);
		vertices[0].weight = u / (u + v);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
	}
}

// Voronoi regions A, B, C, AB, BC, AC, ABC
void Simplex::Solve3()
{
	glm::vec3 A = vertices[0].point;
	glm::vec3 B = vertices[1].point;
	glm::vec3 C = vertices[2].point;

	// Vertex regions
	glm::vec3 AB = B - A;
	glm::vec3 AC = C - A;
	glm::vec3 AO = -A;
	float d1 = glm::dot(AB, AO);
	float d2 = glm::dot(AC, AO);

	// R A
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// R B
	glm::vec3 BO = -B;
	float d3 = glm::dot(AB, BO);
	float d4 = glm::dot(AC, BO);
	if (d3 >= 0.0f && d4 <= d3)
	{
		vertices[0] = vertices[1];
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// R C
	glm::vec3 CO = -C;
	float d5 = glm::dot(AB, CO);
	float d6 = glm::dot(AC, CO);
	if (d6 >= 0.0f && d5 <= d6)
	{
		vertices[0] = vertices[2];
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// Edge regions
	// R AB
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		vertices[0].weight = -d3 / (d1 - d3);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R BC
	float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && d4 >= d3 && d5 >= d6)
	{
		vertices[0] = vertices[1];
		vertices[1] = vertices[2];
		vertices[0].weight = (d5 - d6) / ((d5 - d6) + (d4 - d3));
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R AC
	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		vertices[1] = vertices[2];
		vertices[0].weight = -d6 / (d2 - d6);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// Face region
	// R ABC
	float invDenom = 1.0f / (va + vb + vc);
	vertices[0].weight = va * invDenom;
	vertices[1].weight = vb * invDenom;
	vertices[2].weight = 1.0f - vertices[0].weight - vertices[1].weight;
	nVerts = 3;
}

// 15 Voronoi regions A, B, C, D, AB, BC, CA, CD, DA, BD, ABC, BCD, CDA, DAB, ABCD 
void Simplex::Solve4()
{
	glm::vec3 A = vertices[0].point;
	glm::vec3 B = vertices[1].point;
	glm::vec3 C = vertices[2].point;
	glm::vec3 D = vertices[3].point;

	// Vertex regions

	glm::vec3 AB = B - A;
	glm::vec3 AC = C - A;
	glm::vec3 AD = D - A;
	glm::vec3 AO = -A;
	float vAB = glm::dot(AO, AB);
	float vAC = glm::dot(AO, AC);
	float vAD = glm::dot(AO, AD);

	// R A
	if (vAB <= 0.0f && vAC <= 0.0f && vAD <= 0.0f)
	{
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// R B
	glm::vec3 BO = -B;
	glm::vec3 BC = C - B;
	glm::vec3 DB = B - D;
	float uAB = glm::dot(BO, -AB);
	float vBC = glm::dot(BO, BC);
	float uDB = glm::dot(BO, -DB);
	if (uAB <= 0.0f && vBC <= 0.0f && uDB <= 0.0f)
	{
		vertices[0] = vertices[1];
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// R C
	glm::vec3 CO = -C;
	glm::vec3 CD = D - C;
	float uAC = glm::dot(CO, -AC);
	float uBC = glm::dot(CO, -BC);
	float vCD = glm::dot(CO, CD);
	if (uAC <= 0.0f && uBC <= 0.0f && vCD <= 0.0f)
	{
		vertices[0] = vertices[2];
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// R D
	glm::vec3 DO = -D;
	float uAD = glm::dot(DO, -AD);
	float vDB = glm::dot(DO, DB);
	float uCD = glm::dot(DO, -CD);
	if (uAD <= 0.0f && vDB <= 0.0f && uCD <= 0.0f)
	{
		vertices[0] = vertices[3];
		vertices[0].weight = 1.0f;
		nVerts = 1;
		return;
	}

	// Edge regions

	// R AB
	glm::vec3 A_x_B = glm::cross(A, B);
	glm::vec3 AB_x_AD = glm::cross(AB, AD);
	float wABD = glm::dot(A_x_B, AB_x_AD);
	glm::vec3 AC_x_AB = glm::cross(AC, AB);
	float vACB = glm::dot(-A_x_B, AC_x_AB);
	if (uAB >= 0.0f && vAB >= 0.0f && wABD <= 0.0f && vACB <= 0.0f)
	{
		vertices[0].weight = uAB / (uAB + vAB);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R AC
	glm::vec3 A_x_C = glm::cross(A, C);
	float wACB = glm::dot(A_x_C, AC_x_AB);
	glm::vec3 AD_x_AC = glm::cross(AD, AC);
	float vADC = glm::dot(-A_x_C, AD_x_AC);
	if (uAC >= 0.0f && vAC >= 0.0f && wACB <= 0.0f && vADC <= 0.0f)
	{
		vertices[1] = vertices[2];
		vertices[0].weight = uAC / (uAC + vAC);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R AD
	glm::vec3 A_x_D = glm::cross(A, D);
	float wADC = glm::dot(A_x_D, AD_x_AC);
	float vABD = glm::dot(-A_x_D, AB_x_AD);
	if (uAD >= 0.0f && vAD >= 0.0f && wADC <= 0.0f && vABD <= 0.0f)
	{
		vertices[1] = vertices[3];
		vertices[0].weight = uAD / (uAD + vAD);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R BC
	glm::vec3 B_x_C = glm::cross(B, C);
	float uACB = glm::dot(-B_x_C, AC_x_AB);
	glm::vec3 BC_x_BD = glm::cross(BC, -DB);
	float wBCD = glm::dot(B_x_C, BC_x_BD);
	if (uBC >= 0.0f && vBC >= 0.0f && uACB <= 0.0f && wBCD <= 0.0f)
	{
		vertices[0] = vertices[1];
		vertices[1] = vertices[2];
		vertices[0].weight = uBC / (uBC + vBC);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R CD
	glm::vec3 C_x_D = glm::cross(C, D);
	float uADC = glm::dot(-C_x_D, AD_x_AC);
	float uBCD = glm::dot(C_x_D, BC_x_BD);
	if (uCD >= 0.0f && vCD >= 0.0f && uADC <= 0.0f && uBCD <= 0.0f)
	{
		vertices[0] = vertices[2];
		vertices[1] = vertices[3];
		vertices[0].weight = uCD / (uCD + vCD);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// R DB
	glm::vec3 D_x_B = glm::cross(D, B);
	float uABD = glm::dot(-D_x_B, AB_x_AD);
	float vBCD = glm::dot(D_x_B, BC_x_BD);
	if (uDB >= 0.0f && vDB >= 0.0f && uABD <= 0.0f && vBCD <= 0.0f)
	{
		vertices[0] = vertices[3];
		vertices[0].weight = uDB / (uDB + vDB);
		vertices[1].weight = 1.0f - vertices[0].weight;
		nVerts = 2;
		return;
	}

	// Face regions

	// R ACB
	float det = glm::determinant(glm::mat3(AB, AC, AD));
	float sign = det > 0.0f ? 1.0f : -1.0f;
	float tACB = sign * glm::determinant(glm::mat3(A, C, B));
	if (tACB <= 0.0f && uACB >= 0.0f && vACB >= 0.0f && wACB >= 0.0f)
	{
		float invDenom = 1.0f / (uACB + vACB + wACB);
		vertices[1] = vertices[2];
		vertices[2] = vertices[1];
		vertices[0].weight = uACB * invDenom;
		vertices[1].weight = vACB * invDenom;
		vertices[2].weight = 1.0f - vertices[0].weight - vertices[1].weight;
		nVerts = 3;
		return;
	}

	// R ABD
	float tABD = sign * glm::determinant(glm::mat3(A, B, D));
	if (tABD <= 0.0f && uABD >= 0.0f && vABD >= 0.0f && wABD >= 0.0f)
	{
		float invDenom = 1.0f / (uABD + vABD + wABD);
		vertices[2] = vertices[3];
		vertices[0].weight = uABD * invDenom;
		vertices[1].weight = vABD * invDenom;
		vertices[2].weight = 1.0f - vertices[0].weight - vertices[1].weight;
		nVerts = 3;
		return;
	}

	// R ADC
	float tADC = sign * glm::determinant(glm::mat3(A, D, C));
	if (tADC <= 0.0f && uADC >= 0.0f && vADC >= 0.0f && wADC >= 0.0f)
	{
		float invDenom = 1.0f / (uADC + vADC + wADC);
		vertices[1] = vertices[3];
		vertices[0].weight = uADC * invDenom;
		vertices[1].weight = vADC * invDenom;
		vertices[2].weight = 1.0f - vertices[0].weight - vertices[1].weight;
		nVerts = 3;
		return;
	}

	// R BCD
	float tBCD = sign * glm::determinant(glm::mat3(B, C, D));
	if (tBCD <= 0.0f && uBCD >= 0.0f && vBCD >= 0.0f && wBCD >= 0.0f)
	{
		float invDenom = 1.0f / (uBCD + vBCD + wBCD);
		vertices[0] = vertices[1];
		vertices[1] = vertices[2];
		vertices[2] = vertices[3];
		vertices[0].weight = uBCD * invDenom;
		vertices[1].weight = vBCD * invDenom;
		vertices[2].weight = 1.0f - vertices[0].weight - vertices[1].weight;
		nVerts = 3;
		return;
	}

	// R ABCD
	float invDenom = 1.0f / (tACB + tABD + tADC + tBCD);

	if (invDenom <= 0.0f)
	{
		return;
	}

	vertices[0].weight = tACB * invDenom;
	vertices[1].weight = tABD * invDenom;
	vertices[2].weight = tADC * invDenom;
	vertices[3].weight = tBCD * invDenom;
	nVerts = 4;
}

void Simplex::CalculateClosestPoints(glm::vec3& pA, glm::vec3& pB)
{
	switch (nVerts)
	{
	case 1:
		pA = vertices[0].pointA;
		pB = vertices[0].pointB;
		break;
	case 2:
		pA = vertices[0].weight * vertices[0].pointA + vertices[1].weight * vertices[1].pointA;
		pB = vertices[0].weight * vertices[0].pointB + vertices[1].weight * vertices[1].pointB;
		break;
	case 3:
		pA = vertices[0].weight * vertices[0].pointA + vertices[1].weight * vertices[1].pointA + vertices[2].weight * vertices[2].pointA;
		pB = vertices[0].weight * vertices[0].pointB + vertices[1].weight * vertices[1].pointB + vertices[2].weight * vertices[2].pointB;
		break;
	case 4:
		pA = vertices[0].weight * vertices[0].pointA + vertices[1].weight * vertices[1].pointA + vertices[2].weight * vertices[2].pointA + vertices[3].weight * vertices[3].pointA;
		pB = pA;
		break;
	}
}

void DistanceProxy::SetProxy(Collider* collider)
{
	switch (collider->GetShape())
	{
	case Collider::Sphere:
		vertices.push_back(collider->GetCentroid());
		break;
	case Collider::Hull:
		vertices.reserve(static_cast<HullCollider*>(collider)->GetVertexCount());
		for (HVertex* v : static_cast<HullCollider*>(collider)->GetVertices())
			vertices.push_back(v->position);
		break;
	default:
		assert(false);
		break;
	}
}

int DistanceProxy::GetSupport(const glm::vec3& dir) const
{
	float maxDist = -FLT_MAX;
	float dist = 0;
	int support = -1;

	for (int i = 0; i < vertices.size(); i++)
	{
		dist = glm::dot(vertices[i], dir);
		if (dist > maxDist)
		{
			maxDist = dist;
			support = i;
		}
	}

	return support;
}

glm::vec3 DistanceProxy::GetVertex(int i) const
{
	return vertices[i];
}

std::pair<glm::vec3, glm::vec3> GJKDistance(Collider* A, Collider* B)
{
	Simplex simplex;
	DistanceProxy proxyA, proxyB;

	proxyA.SetProxy(A);
	proxyB.SetProxy(B);

	// Initialize the simplex
	{
		SimplexVertex* v = simplex.vertices;

		// To Do: Check if good initial guess is faster than starting at a random vertex
		//static glm::vec3 dir = glm::vec3(1, 0, 0);	// Initial search direction	// To Do : Need to find a good initial search direction
		//v->indexA = A->hull->GetSupport(A->body->WorldToLocalDir(-dir));
		//v->pointA = A->body->LocalToWorld(A->hull->GetVertex(v->indexA));
		//v->indexB = B->hull->GetSupport(B->body->WorldToLocalDir(dir));
		//v->pointB = B->body->LocalToWorld(B->hull->GetVertex(v->indexB));

		v->indexA = 0;
		v->pointA = A->GetBody()->LocalToGlobalPoint(proxyA.GetVertex(0));
		v->indexB = 0;
		v->pointB = B->GetBody()->LocalToGlobalPoint(proxyB.GetVertex(0));
		v->point = v->pointB - v->pointA;
		v->weight = 1.0f;
		simplex.nVerts = 1;
	}

	SimplexVertex* verts = simplex.vertices;

	// Cache the support vertices from previous iteration
	// Terminate if a repeated support point is found on the simplex
	int prevA[4], prevB[4];
	int nVertsPrev = 0;

	// Cache distance from previous iteration
	// Terminate if we are not getting closer to the origin
	float prevDist2 = FLT_MAX;
	float dist2 = 0.0f;

	const glm::vec3 origin(0, 0, 0);

	// Terminate if taking too many iterations
	int maxIters = 100;

	// GJK loop
	int iter = 0;
	while (iter < maxIters)
	{
		nVertsPrev = simplex.nVerts;
		for (int i = 0; i < nVertsPrev; i++)
		{
			prevA[i] = verts[i].indexA;
			prevB[i] = verts[i].indexB;
		}

		// Find closest point on simplex, and cull unused vertices from the simplex
		switch (simplex.nVerts)
		{
		case 1:
			break;
		case 2:
			simplex.Solve2();
			break;
		case 3:
			simplex.Solve3();
			break;
		case 4:
			simplex.Solve4();
			break;
		default:
			break;
		}

		// If we still have 4 points after reducing, then the origin is inside
		if (simplex.nVerts == 4)
		{
			//return std::pair<glm::vec3, glm::vec3>(glm::vec3(0), glm::vec3(0));	// overlapping?
			break;
		}

		glm::vec3 closest = simplex.FindClosestPoint();
		dist2 = glm::length2(closest);

		const static float epsilon = 0.00001f;
		// Origin is the closest point
		if (dist2 < FLT_EPSILON * FLT_EPSILON)
		{
			//return std::pair<glm::vec3, glm::vec3>(glm::vec3(0), glm::vec3(0));	// overlapping?
			//break;
		}

		// Check if we are getting closer to origin
		if (prevDist2 < dist2)
		{
			break;
		}
		prevDist2 = dist2;

		glm::vec3 dir = simplex.FindSearchDirection();

		// Terminate on bad search direction
		if (glm::dot(dir, dir) < FLT_EPSILON * FLT_EPSILON)
			break;

		SimplexVertex* v = simplex.vertices + simplex.nVerts;
		v->indexA = proxyA.GetSupport(A->GetBody()->GlobalToLocalVec(-dir));
		v->pointA = A->GetBody()->LocalToGlobalPoint(proxyA.GetVertex(v->indexA));
		v->indexB = proxyB.GetSupport(B->GetBody()->GlobalToLocalVec(dir));
		v->pointB = B->GetBody()->LocalToGlobalPoint(proxyB.GetVertex(v->indexB));
		v->point = v->pointB - v->pointA;

		// Main termination condition
		// Check if this newly added vertex matches with any existing vertex in the simplex
		bool duplicate = false;
		for (int i = 0; i < nVertsPrev; i++)
		{
			if (v->indexA == prevA[i] && v->indexB == prevB[i])
			{
				duplicate = true;
				break;
			}
		}

		if (duplicate)
			break;

		simplex.nVerts++;
		iter++;
	}

	glm::vec3 pA(0, 0, 0), pB(0, 0, 0);
	simplex.CalculateClosestPoints(pA, pB);

	return std::pair<glm::vec3, glm::vec3>(pA, pB);
}