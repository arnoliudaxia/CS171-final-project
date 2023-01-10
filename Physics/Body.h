
#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "Model.h"

class Collider;

struct ModelData
{
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;
	std::vector<int> frameIndices;

	ModelData() {};

	ModelData(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices, const std::vector<int>& frameIndices)
		: vertices(vertices), indices(indices), frameIndices(frameIndices)
	{}
};

struct Velocity
{
	glm::vec3 v;
	glm::vec3 w;

	Velocity(const glm::vec3& v, const glm::vec3& w)
		: v(v), w(w)
	{}
};

struct Position
{
	glm::vec3 c;
	glm::quat q;

	Position(const glm::vec3& c, const glm::quat& q)
		: c(c), q(q)
	{}
};

class Body
{
private:
	float invMass;
	glm::mat3 localInertia;
	glm::mat3 localInvInertia;
	glm::mat3 invInertia;

	float density;
	//恢复系数，和弹性碰撞速度损失有关
	float restitution;
	float friction;

	glm::vec3 localCentroid;
	glm::vec3 centroid;
	glm::vec3 position;
	glm::quat orientation;
	glm::mat3 R;

	glm::vec3 velocity;
	glm::vec3 angularVelocity;

	// tracks how much the body has moved in recent frames
	float motion;

	glm::vec3 forceSum;
	glm::vec3 torqueSum;

	std::vector<Collider*> colliders;

	Model* model;
	Model* frame;
	glm::vec3 color;
	int group;

public:
	Body();

	std::vector<Collider*>& GetColliders();

	void SetModelData(const ModelData& m);

	void SetColor(const glm::vec3& color);

	void SetGroup(const int s);
	int GetGroup() const;

	const glm::vec3 LocalToGlobalVec(const glm::vec3& v) const;

	const glm::vec3 GlobalToLocalVec(const glm::vec3& v) const;

	const glm::vec3 LocalToGlobalPoint(const glm::vec3& p) const;

	const glm::vec3 GlobalToLocalPoint(const glm::vec3& p) const;

	const glm::vec3 LocalToLocalVec(Body* A, const glm::vec3& v) const;
	const glm::vec3 LocaltoLocalPoint(Body* A, const glm::vec3& p) const;

	void AddCollider(Collider* collider);

	void ApplyForce(const glm::vec3& force);
	void ApplyForce(const glm::vec3& force, const glm::vec3& pt);

	void IntegratePosition(const float dt);

	glm::vec3 SolveGyroscopic(glm::vec3 w1, float dt);

	void Update(const float dt);

	void Render();
};