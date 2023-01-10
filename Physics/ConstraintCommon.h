
#pragma once

#include <glm/glm.hpp>
#include "Body.h"

// The Jacobian is a transformation b/w world space and constraint space
struct Jacobian
{
	// The coefficients of the linear and angular velocity components,
	// in the velocity constraint equation
	glm::vec3 L1;
	glm::vec3 A1;
	glm::vec3 L2;
	glm::vec3 A2;

	Jacobian() {}

	Jacobian(const glm::vec3& L1, const glm::vec3& A1, const glm::vec3& L2, const glm::vec3& A2)
		: L1(L1), A1(A1), L2(L2), A2(A2)
	{}
};

class Body;

// Generic code used by all constraints
// Effective mass acounts for distributing the impulse (constraint space)
// uniformly for both bodies (linear and angular components)
float CalculateEffectiveMass(const Jacobian& J, const Body* A, const Body* B);

// Lagrangian is the impulse in constraint space to satisfy the constraint
float CalculateLagrangian(const Jacobian& J, const Velocity& A, const Velocity& B, const float effMass, const float bias = 0);

float CalculateLagrangian(const Jacobian& J, const Body*A, const Body* B, const float effMass, const float bias = 0);

// Convert the lagrangian into world space and apply to the bodies
void ApplyImpulse(const Jacobian& J, Body* A, Body* B, const float lambda);
