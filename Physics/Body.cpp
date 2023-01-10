
#pragma once

#include "Collider.h"
#include "Body.h"
#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "HullCollider.h"
#include "SphereCollider.h"
#include "Geometry.h"
#include "../SystemManager.h"

#define GRAVITY 9.8f
#define SLEEP_EPSILON2 0.12f
#define SLEEP_EPSILON_MAX 1.0f
#define MAX_VELOCITY2 400.0f

Body::Body()
	:invMass(1.0),
	localInertia(0),
	localInvInertia(0),
	invInertia(0),
	density(1.0f),
	restitution(0.3f),
	friction(0.4f),
	position(0),
	orientation(1, 0, 0, 0),
	R(1),
	velocity(0),
	angularVelocity(0),
	motion(5.0f * SLEEP_EPSILON2),
	awake(true),
	canSleep(true),
	forceSum(0),
	torqueSum(0),
	color(0.4, 0.9, 0.1),
	group(0)
{}

std::vector<Collider*>& Body::GetColliders()
{
	return colliders;
}

void Body::SetModelData(const ModelData& m)
{
	model = new Model(m.vertices, m.indices);
	if (m.frameIndices.size() != 0)
	{
		frame = new Model(m.vertices, m.frameIndices);
		frame->SetPrimitive(GL_LINES);
	}
	else
		frame = nullptr;
}

void Body::SetColor(const glm::vec3& color)
{
	this->color = color;
}

void Body::SetGroup(const int g)
{
	group = g;
}

int Body::GetGroup() const
{
	return group;
}

const glm::vec3 Body::LocalToGlobalVec(const glm::vec3& v) const
{
	return R * v;
}

const glm::vec3 Body::GlobalToLocalVec(const glm::vec3& v) const
{
	return glm::transpose(R) * v;
}

const glm::vec3 Body::LocalToGlobalPoint(const glm::vec3& p) const
{
	return position + R * p;
}

const glm::vec3 Body::GlobalToLocalPoint(const glm::vec3& p) const
{
	return glm::transpose(R) * (p - position);
}

const glm::vec3 Body::LocalToLocalVec(Body* A, const glm::vec3& v) const
{
	return GlobalToLocalVec(A->LocalToGlobalVec(v));
}

const glm::vec3 Body::LocaltoLocalPoint(Body* A, const glm::vec3& p) const
{
	return GlobalToLocalPoint(A->LocalToGlobalPoint(p));
}

void Body::AddCollider(Collider* collider)
{
	collider->SetBody(this);
	colliders.push_back(collider);

	if (invMass == 0) return;

	localCentroid = glm::vec3(0.0f);
	invMass = 0.0f;

	switch (collider->GetShape())
	{
	case (Collider::Hull):
		static_cast<HullCollider*>(collider)->CalculateMass();
		break;
	case(Collider::Sphere):
		static_cast<SphereCollider*>(collider)->CalculateMass();
		break;
	}

	float mass = 0.0f;		// mass of the body

	for (Collider* c : colliders)
	{
		mass += c->GetMass();

		localCentroid += c->GetMass() * c->GetCentroid();
	}

	assert(mass != 0);

	invMass = 1.0f / mass;

	localCentroid *= invMass;
	centroid = LocalToGlobalPoint(localCentroid);

	for (Collider* c : colliders)
	{
		glm::vec3 r = localCentroid - c->GetCentroid();
		float rDotr = glm::dot(r, r);
		glm::mat3 rOutr = glm::outerProduct(r, r);

		// Parallel axis theorem
		// Accumulate local inertia tensors
		localInertia += c->GetInertia() + c->GetMass() * (rDotr * glm::mat3(1.0) - rOutr);
	}

	localInvInertia = glm::inverse(localInertia);
}

void Body::ApplyForce(const glm::vec3& force)
{
	forceSum += force;
}

void Body::ApplyForce(const glm::vec3& force, const glm::vec3& p)
{
	forceSum += force;
	torqueSum += glm::cross(LocalToGlobalPoint(p) - centroid, force);
}

void Body::IntegratePosition(const float dt)
{
	centroid += velocity * dt;
	orientation += 0.5f * glm::quat(0, angularVelocity) * orientation * dt;

	orientation = glm::normalize(orientation);
	R = glm::toMat3(orientation);
	position = centroid - LocalToGlobalVec(localCentroid);
}


void Body::Update(const float dt)
{
	if (invMass == 0.0)
		return;
	if (position.y < -100.0f)
	{
		//Object is fallen down out of scene
		awake = false;
	}
	if (!awake)
		return;

	if (!shouldStop)
	{
		forceSum += glm::vec3(0.f, -GRAVITY, 0.f) * GetMass();

	}
	else
	{
		forceSum += glm::vec3(0.f, -GRAVITY, 0.f) * GetMass() * 0.1f;
	}
	//torqueSum += glm::vec3(0.f, -GRAVITY, 0.f) * 0.5f;
	invInertia = glm::transpose(R) * localInvInertia * R;
	velocity += forceSum * dt / GetMass();

	angularVelocity += invInertia * torqueSum * dt;

	forceSum = glm::vec3(0);
	torqueSum = glm::vec3(0);

	centroid += velocity * dt;
	orientation += 0.5f * orientation * glm::quat(0.0f, angularVelocity) * dt;

	orientation = glm::normalize(orientation);
	R = glm::toMat3(orientation);

	position = centroid - LocalToGlobalVec(localCentroid);

	// damping
	velocity *= 0.99f;
	angularVelocity *= 0.99f;
}

void Body::Render()
{
	static glm::mat4 T(1), R(1), S(1), M(1), V(1), P(1), VP(1), MVP(1);
	V = Camera::GetInstance().GetViewMatrix();
	P = Camera::GetInstance().GetProjectionMatrix();

	T = glm::translate(position);
	R = glm::toMat4(orientation);
	M = T * R * S;
	VP = P * V;
	MVP = VP * M;
	model->SetMVP(MVP);
	model->SetColor(color);
	model->Render();

	if (frame != nullptr)
	{
		frame->SetMVP(MVP);
		frame->SetColor(glm::vec3(0.9));
		frame->Render();
	}
}