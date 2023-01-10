
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Particle.h"

class Collider;
class SphereCollider;
class HFace;
struct AABB;

// true if point is inside
bool QueryPoint(Collider* collider, const glm::vec3& point);

// creates particle contact if it is inside
void QueryPoint(std::vector<ParticleContact>& contacts, Collider* collider, Particle* p);

// returns true if segment intersects the sphere 
// computes P as the midpoiint of the intersected segment
bool IntersectSegmentSphere(const glm::vec3& A, const glm::vec3& B, SphereCollider* s, glm::vec3& P);

//	true of the points form a CCW triangle w.r.t given normal
bool TriangleIsCCW(const glm::vec3& A, const glm::vec3& B, const glm::vec3& C, const glm::vec3& normal);

// true if point is inside polygon defined by the vertices
bool QueryPoint(const glm::vec3& P, const std::vector<glm::vec3>& verts, const glm::vec3& normal);

// true if AABBs overlap
bool Overlap(AABB* A, AABB* B);