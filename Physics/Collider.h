
#pragma once

#include <glm/glm.hpp>

class Body;

class Collider
{
public:
	enum Shape
	{
		Sphere = 1,
		Hull,
	};
protected:
	float mass;
	glm::mat3 inertia;

	Shape shape;
	glm::vec3 centroid;
	Body* body;

public:
	Collider()
		:mass(0),
		inertia(0),
		centroid(0)
	{}

	float GetMass() const { return mass; };

	glm::mat3 GetInertia() const { return inertia; };

	Shape GetShape() const { return shape; };

	glm::vec3 GetCentroid() const { return centroid; };

	void SetBody(Body* b) { body = b; }
	Body* GetBody() const { return body; };
};