
#pragma once

#include <glm/glm.hpp>
#include <utility>
#include <vector>

class Collider;
class Model;

struct AABB
{
	glm::vec3 min;
	glm::vec3 max;
	Collider* collider;

	AABB(const glm::vec3& min, const glm::vec3& max, Collider* c)
		: min(min), max(max), collider(c)
	{}
};

class BroadPhase
{
public:
	static BroadPhase& GetInstance();

	void Init(std::vector<Collider*>& colliders);

	std::vector<std::pair<Collider*, Collider*>>& ComputePairs();

	// recalculate AABBs
	void Update();

	void Render();

private:
	BroadPhase();
	
	// local aabbs calculated once (and updated whenever bodies are added or removed)
	std::vector<AABB*> aabbsL;
	std::vector<AABB*> aabbs;
	std::vector<std::pair<Collider*, Collider*>> colliderPairs;
	Model* box;
};