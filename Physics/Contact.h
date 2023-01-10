
#pragma once

#include <glm/glm.hpp>
#include "Geometry.h"
#include "ConstraintCommon.h"
#include "Body.h"


class Contact
{
private:
	Body* A;
	Body* B;

	glm::vec3 position;		// contact point in world space
	glm::vec3 normal;
	glm::vec3 tangent[2];
	float penetration;		//穿深

	float impulseSumN;
	float impulseSumT[2];

	glm::vec3 rA;
	glm::vec3 rB;

	Jacobian JN;
	Jacobian JT[2];

	float bias;

	float kn;
	float kt[2];

private:
	// Projection of relative velocity along the normal 
	float CalculateNormalConstraint() const;

	// Calculates the Jacobian for the normal/tangent part
	void CalculateJacobian(Jacobian& J, const glm::vec3& axis);

	void CalculateBias();

public:
	Contact(Body* A, Body* B, const glm::vec3& position, const glm::vec3& normal, const float penetration);

	glm::vec3 GetPosition() const { return position; }
	Body* GetBodyA() const { return A; }
	Body* GetBodyB() const { return B; }

	glm::vec3 GetTangent(int i) const { return tangent[i]; }

	// Solves the contact by applying impulses
	void SolveVelocities(Velocity& A, Velocity& B);

	void SolvePositions(Position& A, Position& B);

	friend struct Manifold;
};

struct Manifold
{
	std::vector<Contact> contacts;	// To Do : Duplicated data. Store pointer to contact and contact size

	void SolveVelocities();

	void SolvePositions();
};