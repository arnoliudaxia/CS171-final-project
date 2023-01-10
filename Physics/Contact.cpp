
#include "Contact.h"
#include "../SystemManager.h"

#define BAUMGARTE 0.2f
#define VELOCITYSLOP 0.5f
#define POSITIONSLOP 0.001f

//在Contact初始化的时候，需要同时初始化两个constrain相关的东西
Contact::Contact(Body* A, Body* B, const glm::vec3& position, const glm::vec3& normal, const float penetration)
	: A(A), B(B), position(position), normal(normal), penetration(penetration), impulseSumN(0.0f)
{
	ComputeBasis(normal, tangent[0], tangent[1]);

	rA = position - A->GetCentroid();
	rB = position - B->GetCentroid();
	//计算normal constrain的Jacobian、bias和有效质量
	CalculateJacobian(JN, normal);
	CalculateBias();
	kn = CalculateEffectiveMass(JN, A, B);
	//计算Friction constrain的Jacobian有效质量
	for (int i = 0; i < 2; i++)
	{
		impulseSumT[i] = 0.0f;
		CalculateJacobian(JT[i], tangent[i]);
		kt[i] = CalculateEffectiveMass(JT[i], A, B);
	}
}
//Normal constraint(OK)
float Contact::CalculateNormalConstraint() const
{
	glm::vec3 vA = A->GetVelocity() + glm::cross(A->GetAngularVelocity(), rA);
	glm::vec3 vB = B->GetVelocity() + glm::cross(B->GetAngularVelocity(), rB);
	return glm::dot((vB - vA), normal);
}
//3-Compute the Jacobian J for non-penetration and friction constraints(OK)
void Contact::CalculateJacobian(Jacobian& J, const glm::vec3& axis)
{
	J = Jacobian(-axis, -glm::cross(rA, axis), axis, glm::cross(rB, axis));
}
//Slop bias(OK)
void Contact::CalculateBias()
{
	// restitution
	float e = (A->GetRestitution() + B->GetRestitution()) * 0.5f;
	float vSep = CalculateNormalConstraint();
	//bias = e * vSep;
	bias = e * (glm::min(vSep - 0.5f, 0.0f));
}

void Contact::SolveVelocities(Velocity& vA, Velocity& vB)
{
	if (CalculateNormalConstraint() >= 0)
	{
		return;
	}

	float lambda, oldImpulse;
	float uf = (A->GetFriction() + B->GetFriction()) * 0.5f;
	float maxFriction;

	lambda = CalculateLagrangian(JN, vA, vB, kn, bias);

	oldImpulse = impulseSumN;
	impulseSumN = glm::max(0.0f, impulseSumN + lambda);
	lambda = impulseSumN - oldImpulse;

	vA.v += lambda * JN.L1 * A->GetInvMass();
	vA.w += lambda * JN.A1 * A->GetInvInertia();
	vB.v += lambda * JN.L2 * B->GetInvMass();
	vB.w += lambda * JN.A2 * B->GetInvInertia();

	for (int i = 0; i < 2; i++)
	{
		lambda = CalculateLagrangian(JT[i], vA, vB, kt[i], 0.0f);

		oldImpulse = impulseSumT[i];
		maxFriction = uf * impulseSumN; //uf * kt[i] * 9.8f;
		impulseSumT[i] = glm::clamp(impulseSumT[i] + lambda, -maxFriction, maxFriction);
		lambda = impulseSumT[i] - oldImpulse;

		vA.v += lambda * JT[i].L1 * A->GetInvMass();
		vA.w += lambda * JT[i].A1 * A->GetInvInertia();
		vB.v += lambda * JT[i].L2 * B->GetInvMass();
		vB.w += lambda * JT[i].A2 * B->GetInvInertia();
	}
}

void Contact::SolvePositions(Position& pA, Position& pB)
{
	float K = CalculateEffectiveMass(JN, A, B);
	float C = -BAUMGARTE * (glm::max(penetration - POSITIONSLOP, 0.0f));
	float lambda = K > 0.0f ? -C / K : 0.0f;
	glm::vec3 P = lambda * normal;

	pA.c -= P * A->GetInvMass();
	pA.q += 0.5f * glm::quat(0.0f, A->GetInvInertia() * glm::cross(rA, -P)) * pA.q;
	pA.q = glm::normalize(pA.q);
	pB.c += P * B->GetInvMass();
	pB.q += 0.5f * glm::quat(0.0f, B->GetInvInertia() * glm::cross(rB, P)) * pA.q;
	pB.q = glm::normalize(pB.q);
}

void Manifold::SolveVelocities()
{
	Body* A = contacts[0].A;
	Body* B = contacts[0].B;

	if (A->GetInvMass() != 0 && B->GetInvMass() != 0)
	{
		if (!A->IsAwake() || !B->IsAwake())
		{
			B->SetAwake(true);
			A->SetAwake(true);
		}
	}

	glm::vec3 v1 = A->GetVelocity();
	glm::vec3 w1 = A->GetAngularVelocity();
	glm::vec3 v2 = B->GetVelocity();
	glm::vec3 w2 = B->GetAngularVelocity();
	Velocity vA(v1, w1);
	Velocity vB(v2, w2);

	for (int i = 0; i < contacts.size(); i++)
	{
		contacts[i].SolveVelocities(vA, vB);
	}

	A->SetVelocity(vA.v);
	A->SetAngularVelocity(vA.w);
	B->SetVelocity(vB.v);
	B->SetAngularVelocity(vB.w);
}

void Manifold::SolvePositions()
{
	Body* A = contacts[0].A;
	Body* B = contacts[0].B;
	glm::vec3 c1 = A->GetCentroid();
	glm::quat q1 = A->GetOrientation();
	glm::vec3 c2 = B->GetCentroid();
	glm::quat q2 = B->GetOrientation();
	Position pA(c1, q1);
	Position pB(c2, q2);

	for (int i = 0; i < contacts.size(); i++)
	{
		contacts[i].SolvePositions(pA, pB);
	}

	A->SetCentroid(pA.c);
	A->SetOrientation(pA.q);
	B->SetCentroid(pB.c);
	B->SetOrientation(pB.q);
}