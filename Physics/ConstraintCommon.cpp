
#include "ConstraintCommon.h"
//计算有效质量(OK)
float CalculateEffectiveMass(const Jacobian& J, const Body* A, const Body* B)
{
	// effMass = (J)*(invMassMatrix)*(invJ)
	// effMass = L1.L1*im1 + A1.II1*A1 + L2*L2*im2 + A2.II2*A2
	return	(glm::dot(J.L1, J.L1) * A->GetInvMass()
		+ glm::dot(J.A1, J.A1 * A->GetInvInertia())
		+ glm::dot(J.L2, J.L2) * B->GetInvMass()
		+ glm::dot(J.A2, J.A2 * B->GetInvInertia()));
}
//计算拉格朗日常数（即lambda）
float CalculateLagrangian(const Jacobian& J, const Velocity& A, const Velocity& B, float effMass, float bias)
{
	if (effMass == 0.0f)
		return 0.0f;

	float JVi;	// velocity transformed into constraint space

	JVi = glm::dot(J.L1, A.v)
		+ glm::dot(J.A1, A.w)
		+ glm::dot(J.L2, B.v)
		+ glm::dot(J.A2, B.w);

	return (-(JVi + bias) / effMass);
}

float CalculateLagrangian(const Jacobian& J, const Body* A, const Body* B, const float effMass, const float bias)
{
	if (effMass == 0.0f)
		return 0.0f;

	float JVi;	// velocity transformed into constraint space

	JVi = glm::dot(J.L1, A->GetVelocity())
		+ glm::dot(J.A1, A->GetAngularVelocity())
		+ glm::dot(J.L2, B->GetVelocity())
		+ glm::dot(J.A2, B->GetAngularVelocity());

	return (-(JVi + bias) / effMass);
}

void ApplyImpulse(const Jacobian& J, Body* A, Body* B, float lambda)
{
	glm::vec3 v1 = A->GetVelocity();
	glm::vec3 w1 = A->GetAngularVelocity();
	glm::vec3 v2 = B->GetVelocity();
	glm::vec3 w2 = B->GetAngularVelocity();

	v1 += lambda * J.L1 * A->GetInvMass();
	w1 += lambda * J.A1 * A->GetInvInertia();
	v2 += lambda * J.L2 * B->GetInvMass();
	w2 += lambda * J.A2 * B->GetInvInertia();

	A->SetVelocity(v1);
	A->SetAngularVelocity(w1);
	B->SetVelocity(v2);
	B->SetAngularVelocity(w2);
}