
#include "Geometry.h"
#include <glm/gtc/constants.hpp>

#define EPSILON 0.005f


HalfSpace::HalfSpace(const glm::vec3& normal, float distance)
	: normal(normal), distance(distance)
{}

HalfSpace::HalfSpace(const glm::vec3& normal, const glm::vec3& point)
	: normal(normal), distance(glm::dot(point, normal))
{}

glm::vec3 HalfSpace::Origin() const
{
	return distance * normal;
}

float HalfSpace::Distance(const glm::vec3& point) const
{
	return glm::dot(point, normal) - distance;
}

glm::vec3 HalfSpace::Projection(const glm::vec3& point) const
{
	return point - Distance(point) * normal;
}

bool HalfSpace::Infront(const glm::vec3& point) const
{
	return Distance(point) > 0.0f;
}

bool HalfSpace::Behind(const glm::vec3& point) const
{
	return Distance(point) < 0.0f;
}

bool HalfSpace::On(const glm::vec3& point) const
{
	float dist = Distance(point);

	return (dist < EPSILON&& dist > -EPSILON);
}

//Magic 这个是从normal算出两个正交的向量（给摩擦力用的）
void ComputeBasis(const glm::vec3& i, glm::vec3& j, glm::vec3& k)
{
	if (std::abs(i.x) >= 0.57735f)
		j = glm::vec3(i.y, -i.x, 0);
	else
		j = glm::vec3(0, i.z, -i.y);

	j = glm::normalize(j);
	k = glm::cross(i, j);
	k = glm::normalize(k);
}

void CreateSphere(const float radius, ModelData& model)
{
	model.vertices.clear();
	model.indices.clear();
	model.frameIndices.clear();

	const static int slices = 20;
	const static int stacks = 20;
	float const R = 1.0f / (float)(slices - 1);
	float const S = 1.0f / (float)(stacks - 1);
	const static float PI = glm::pi<float>();

	for (size_t r = 0; r < slices; ++r)
	{
		for (size_t s = 0; s < stacks; ++s)
		{
			float const y = (float)(sin(-PI * 0.5f + PI * r * R));
			float const x = (float)(cos(2.0f * PI * s * S) * sin(PI * r * R));
			float const z = (float)(sin(2.0f * PI * s * S) * sin(PI * r * R));

			model.vertices.push_back(glm::vec3(x, y, z) * radius);
			int curRow = r * stacks;
			int nextRow = (r + 1) * stacks;
			model.indices.push_back(curRow + s);
			model.indices.push_back(nextRow + s);
			model.indices.push_back(nextRow + (s + 1));
			model.indices.push_back(curRow + s);
			model.indices.push_back(nextRow + (s + 1));
			model.indices.push_back(curRow + (s + 1));
		}
	}
}
