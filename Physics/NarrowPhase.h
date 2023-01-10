
#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Contact.h"
#include "HullCollider.h"
#include "SphereCollider.h"

// detect collision b/w colliders
// and generate detailed collision data

// Collision info for nearest face features b/w 2 colliders A and B
struct FaceQuery
{
	float separation;	// negative values indicate point of B inside face of A

	int faceIndex;		// index of face of collider A

	int vertIndex;		// index of best vertex of collider B (min penetrating vertex)
};

// Collision info for nearest edge features b/w 2 colliders A and B
struct EdgeQuery
{
	float separation;	// negative values indicate overlap b/w edges

	int edgeIndex1;		// indices of edges of A and B
	int edgeIndex2;

	glm::vec3 normal;	// parpendicular to both edges, and pointing from A to B
};

// SAT
// Project hulls onto every possible separating axis
// and check for overlap of projections

// Axes are face normals of A
// Retruns true if there is overlap on every face axis
// Store minimizing(least penetrating) vertex of B inside A, and assosiated face of A
bool QueryFaceAxes(FaceQuery& query, HullCollider* A, HullCollider* B);

// Axes are all possible cross products of edge combinations (edgeA, edgeB)
// Edge Pruing - Guass Map Optimization - Dirk Gregorius - GDC 2013
// The possible separating axes are the edges that form a face on the Minkowski difference (A-B)
// The edges found to be intersecting on the gauss map are the supporting edges - so no need to check projections
bool QueryEdgeAxes(EdgeQuery& query, HullCollider* A, HullCollider* B);

// @params A, B, C, D are faces of hull that represent points on the Gauss Map
// The arcs connecting the faces represent the edges
// Two edges form a face on the Minkowski difference if their associated arcs intersect
// A-B is arc1(edge from hull A) and C-D is arc2(edge from hull B)
bool IsMinkowskiFace(const glm::vec3& A, const glm::vec3& B, const glm::vec3& B_x_A, const glm::vec3& C, const glm::vec3& D, const glm::vec3& D_x_C);

void CreateEdgeContact(std::vector<Manifold>& manifolds, HullCollider* A, HullCollider* B, const EdgeQuery& query);

// the most anti-parallel face of incident hull compared to reference face
int FindIncidentFace(HullCollider* incident, HullCollider* reference, int referenceFace);

// create contact patch by clipping incident face against reference face
// Sutherland Hodgman - Ref: Orange book
void CreateFaceContact(std::vector<Manifold>& manifolds, HullCollider* incident, HullCollider* reference, int incidentFace, int referenceFace);

void DetectHullVsHull(std::vector<Manifold>& manifolds, HullCollider* A, HullCollider* B);

void DetectSphereWithSphere(std::vector<Manifold>& manifolds, SphereCollider* A, SphereCollider* B);

void DetectSphereWithHull(std::vector<Manifold>& manifolds, SphereCollider* A, HullCollider* B);

void DetectCollision(std::vector<Manifold>& manifolds, Collider* A, Collider* B);

// GJK to find the closest points of approach between two non-overlapping convex shapes
// Note: This can be used later as a pre-step before running SAT to optimise narrowphase collision detection 
static std::pair<glm::vec3, glm::vec3> GJKDistance(Collider* A, Collider* B);

// Helper to hold vertex data (subset of the shape)
// Only vertices inside DistanceProxy are considered for distance computation
struct DistanceProxy
{
private:
	std::vector<glm::vec3> vertices;

public:
	void SetProxy(Collider* collider);

	int GetSupport(const glm::vec3& dir) const;

	glm::vec3 GetVertex(int i) const;
};

// A vertex on the simplex
struct SimplexVertex
{
	glm::vec3 pointA;	// support on collider A
	glm::vec3 pointB;	// support on collider B
	glm::vec3 point;	// support on the minkowski sum
	float weight;		// barycentric weight of the point
	int indexA;			// index of point A on collider A
	int indexB;			// index of point B on collider B
};

// A generic simplex (can be point, line, triangle, or tetrahedron)
struct Simplex
{
	SimplexVertex vertices[4];	// simplex vertices
	int nVerts;					// number of vertices on the simplex (1 - 4)

	// Find a search direction to evolve the simplex
	glm::vec3 FindSearchDirection() const;

	// Find closest point on the simplex to the origin
	glm::vec3 FindClosestPoint() const;

	// Calculate closest points on the colliders, using the barycentric weights of simplex vertices
	void CalculateClosestPoints(glm::vec3& pA, glm::vec3& pB);

	// Calculate the barycentric weights of the closest point on simplex to origin
	// Find the Voronoi region of the origin, and reduce the simplex if required
	void Solve2();	// Simplex is a segment
	void Solve3();	// Simplex is a triangle
	void Solve4();	// Simplex is a tetrahedron
};