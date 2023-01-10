
#include "BroadPhase.h"
#include "PrimitiveQuery.h"
#include "Collider.h"
#include "Body.h"
#include "SphereCollider.h"
#include "HullCollider.h"
#include "ObjParser.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "Camera.h"
#include "Model.h"

BroadPhase::BroadPhase()
{}

BroadPhase& BroadPhase::GetInstance()
{
	static BroadPhase instance;
	return instance;
}

void BroadPhase::Init(std::vector<Collider*>& colliders)
{
	glm::vec3 min(0), max(0), X(0), Y(0), Z(0);
	int i = 0;
	aabbs.clear();
	aabbsL.clear();
	for (Collider* c : colliders)
	{
		switch (c->GetShape())
		{
		case (Collider::Sphere):
			max = glm::vec3(static_cast<SphereCollider*>(c)->GetRadius());
			min = -max;
			aabbs.push_back(new AABB(min, max, c));
			aabbsL.push_back(new AABB(min, max, c));
			break;
		case (Collider::Hull):
			X = c->GetBody()->GlobalToLocalVec(glm::vec3(1.0, 0, 0));
			Y = c->GetBody()->GlobalToLocalVec(glm::vec3(0, 1.0, 0));
			Z = c->GetBody()->GlobalToLocalVec(glm::vec3(0, 0, 1.0));
			i = static_cast<HullCollider*>(c)->GetSupport(-X);
			min.x = static_cast<HullCollider*>(c)->GetVertex(i)->position.x;
			i = static_cast<HullCollider*>(c)->GetSupport(-Y);
			min.y = static_cast<HullCollider*>(c)->GetVertex(i)->position.y;
			i = static_cast<HullCollider*>(c)->GetSupport(-Z);
			min.z = static_cast<HullCollider*>(c)->GetVertex(i)->position.z;
			i = static_cast<HullCollider*>(c)->GetSupport(X);
			max.x = static_cast<HullCollider*>(c)->GetVertex(i)->position.x;
			i = static_cast<HullCollider*>(c)->GetSupport(Y);
			max.y = static_cast<HullCollider*>(c)->GetVertex(i)->position.y;
			i = static_cast<HullCollider*>(c)->GetSupport(Z);
			max.z = static_cast<HullCollider*>(c)->GetVertex(i)->position.z;

			if (c->GetBody()->GetInvMass() == 0)
			{
				glm::vec3 v[8];
				v[0] = glm::vec3(min.x, min.y, min.z);
				v[1] = glm::vec3(max.x, min.y, min.z);
				v[2] = glm::vec3(max.x, max.y, min.z);
				v[3] = glm::vec3(min.x, max.y, min.z);
				v[4] = glm::vec3(min.x, max.y, max.z);
				v[5] = glm::vec3(max.x, max.y, max.z);
				v[6] = glm::vec3(max.x, min.y, max.z);
				v[7] = glm::vec3(min.x, min.y, max.z);

				for (int i = 0; i < 8; i++)
				{
					v[i] = c->GetBody()->LocalToGlobalPoint(v[i]);
				}

				min = glm::vec3(FLT_MAX);
				max = glm::vec3(-FLT_MAX);
				for (int i = 0; i < 8; i++)
				{
					min.x = glm::min(min.x, v[i].x);
					min.y = glm::min(min.y, v[i].y);
					min.z = glm::min(min.z, v[i].z);
					max.x = glm::max(max.x, v[i].x);
					max.y = glm::max(max.y, v[i].y);
					max.z = glm::max(max.z, v[i].z);
				}
				aabbs.push_back(new AABB(min, max, c));
			}
			else
			{
				aabbs.push_back(new AABB(min, max, c));
				aabbsL.push_back(new AABB(min, max, c));
			}
			break;
		default:
			assert(false);
			break;
		}
	}

	// sort aabbs such that all the static bodies are at the end
	std::vector<AABB*> aabbsStatic;
	std::vector<AABB*> aabbsDynamic;
	for (AABB* aabb : aabbs)
	{
		if (aabb->collider->GetBody()->GetInvMass() == 0)
			aabbsStatic.push_back(aabb);
		else
			aabbsDynamic.push_back(aabb);
	}
	aabbs.clear();
	for (AABB* aabb : aabbsDynamic)
		aabbs.push_back(aabb);
	for (AABB* aabb : aabbsStatic)
		aabbs.push_back(aabb);

	HMesh mesh;
	ModelData boxModel;
	ParseObj("resources/box.obj", mesh);
	mesh.GetModelData(boxModel);
	box = new Model(boxModel.vertices, boxModel.frameIndices);
	box->SetPrimitive(GL_LINES);
	box->SetLineWidth(5.0f);
}

std::vector<std::pair<Collider*, Collider*>>& BroadPhase::ComputePairs()
{
	colliderPairs.clear();
#pragma omp parallel for schedule(dynamic,1)
	for (int iA = 0; iA < aabbs.size(); iA++)
	{
		for (int iB = iA + 1; iB < aabbs.size(); iB++)
		{
			if (aabbs[iA]->collider->GetBody()->GetGroup() != 0)
			{
				if (aabbs[iA]->collider->GetBody()->GetGroup() == aabbs[iB]->collider->GetBody()->GetGroup())
					continue;
			}

			if (aabbs[iA]->collider->GetBody()->GetInvMass() == 0.0f && aabbs[iB]->collider->GetBody()->GetInvMass() == 0.0f)
				continue;

			if (Overlap(aabbs[iA], aabbs[iB]))
				colliderPairs.push_back(std::make_pair(aabbs[iA]->collider, aabbs[iB]->collider));
		}
	}

	return colliderPairs;
}

void BroadPhase::Update()
{
	glm::vec3 v[8];
	for (int i = 0; i < aabbsL.size(); i++)
	{
		AABB* aabbL = aabbsL[i];

		if (aabbL->collider->GetShape() == Collider::Sphere)
		{
			aabbs[i]->min = aabbL->min + aabbs[i]->collider->GetBody()->GetPosition();
			aabbs[i]->max = aabbL->max + aabbs[i]->collider->GetBody()->GetPosition();
			continue;
		}

		v[0] = glm::vec3(aabbL->min.x, aabbL->min.y, aabbL->min.z);
		v[1] = glm::vec3(aabbL->max.x, aabbL->min.y, aabbL->min.z);
		v[2] = glm::vec3(aabbL->max.x, aabbL->max.y, aabbL->min.z);
		v[3] = glm::vec3(aabbL->min.x, aabbL->max.y, aabbL->min.z);
		v[4] = glm::vec3(aabbL->min.x, aabbL->max.y, aabbL->max.z);
		v[5] = glm::vec3(aabbL->max.x, aabbL->max.y, aabbL->max.z);
		v[6] = glm::vec3(aabbL->max.x, aabbL->min.y, aabbL->max.z);
		v[7] = glm::vec3(aabbL->min.x, aabbL->min.y, aabbL->max.z);

		for (int i = 0; i < 8; i++)
		{
			v[i] = aabbL->collider->GetBody()->LocalToGlobalPoint(v[i]);
		}

		AABB* aabb = aabbs[i];
		aabb->min = glm::vec3(FLT_MAX);
		aabb->max = glm::vec3(-FLT_MAX);
		for (int i = 0; i < 8; i++)
		{
			aabb->min.x = glm::min(aabb->min.x, v[i].x);
			aabb->min.y = glm::min(aabb->min.y, v[i].y);
			aabb->min.z = glm::min(aabb->min.z, v[i].z);
			aabb->max.x = glm::max(aabb->max.x, v[i].x);
			aabb->max.y = glm::max(aabb->max.y, v[i].y);
			aabb->max.z = glm::max(aabb->max.z, v[i].z);
		}
	}
}

void BroadPhase::Render()
{
	static glm::mat4 T(1), S(1), M(1);
	glm::mat4 V = Camera::GetInstance().GetViewMatrix();
	glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();

	for (AABB* aabb : aabbs)
	{
		T = glm::translate((aabb->min + aabb->max) * 0.5f);
		S = glm::scale(glm::abs(aabb->max - aabb->min) * 0.5f);
		M = T * S;
		box->SetMVP(P * V * M);
		box->Render();
	}
}