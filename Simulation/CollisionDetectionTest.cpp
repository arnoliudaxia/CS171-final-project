
#include "CollisionDetectionTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BroadPhase.h"
#include "../SystemManager.h"
#include <Camera.h>

const bool HIGHDEMAND = false;

CollisionDetectionTest& CollisionDetectionTest::GetInstance()
{
	static CollisionDetectionTest instance;
	return instance;
}

float myrandom()
{
	return (float)rand() / (float)RAND_MAX;
}
bool checkvaild(std::vector<glm::vec3>& history, glm::vec3 input)
{
	for (auto& position : history)
	{
		if (glm::l2Norm(input, position) < 2.f)
		{
			return false;
		}
	}
	return true;
}
glm::vec3 generatePositon()
{
	static std::vector<glm::vec3> history;

Regen:
	auto positon = glm::vec3(10 + myrandom() * 20, 30 + (myrandom()) * 200, myrandom() * 15);
	if (!checkvaild(history, positon))
	{
		goto Regen;
	}
	return positon;
}

void CollisionDetectionTest::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ModelData model;
	Collider* collider;
	Body body;

	// slope
	Simulation::AddObjToScene("resources/floor.obj",
		glm::vec3(25, 20, 0),
		glm::angleAxis(45.0f / 180 * glm::pi<float>(), glm::vec3(0, 0, 1)),
		0.0f,
		glm::vec3(0.7, 0.7, 0.6));
	// floor
	Simulation::AddObjToScene("resources/floor.obj",
		glm::vec3(-5, -10, 0),
		glm::angleAxis(30.f / 180 * glm::pi<float>(), glm::vec3(0, 0, -1)),
		0.0f,
		glm::vec3(0.7, 0.7, 0.1), 0.3f, 2.f);

	srand(time(NULL));
	int howmany = 0;

	//Simulation::AddObjToScene("resources/cylinder.obj",
	//	generatePositon(),
	//	glm::angleAxis(0.f, glm::vec3(0, 0, -1)),
	//	1.0f,
	//	glm::vec3(0.7, 0.7, 0.1), 0.3f, 2.f);
	if (!genBox)
		goto BallSimu;
	for (int i = 0; i < genBoxNumber; i++)
	{
		Simulation::AddObjToScene("resources/box.obj",
			generatePositon(),
			glm::angleAxis(0.0f, glm::vec3(0, 0, 1)),
			.2f,
			(glm::vec3(250.0 / 255, 193.7 / 255, 26.1 / 255)), 0.2f, 2.f);
		howmany++;
	}
	std::cout << "生成" << howmany << "个boxes" << std::endl;
	howmany = 0;


BallSimu:
	if (!genBall)
		goto END;
	float radius = 1.3f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetMass(0.8f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetRestitution(1.f);
	for (int i = 0; i < genBallNumber; i++)
	{
		body.SetPosition(generatePositon());
		collider = new SphereCollider(radius);
		bodies.push_back(body);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
		howmany++;
	}
	std::cout << "生成" << howmany << "个balls" << std::endl;


END:
	BroadPhase::GetInstance().Init(colliders);
	Camera::GetInstance().Rotate(0, -30, 0);
}

void CollisionDetectionTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	glm::vec3 p = bodies[1].GetPosition();
	glm::quat o = bodies[1].GetOrientation();
	glm::vec3 axis;
	glm::quat dq;
	float t = .5f;
	if (keys[GLFW_KEY_DOWN])
		bodies[1].SetPosition(p + t * glm::vec3(0, -1.0, 0));
	if (keys[GLFW_KEY_UP])
		bodies[1].SetPosition(p + t * glm::vec3(0, 1.0, 0));
	if (keys[GLFW_KEY_RIGHT])
		bodies[1].SetPosition(p + t * glm::vec3(1.0, 0, 0));
	if (keys[GLFW_KEY_LEFT])
		bodies[1].SetPosition(p + t * glm::vec3(-1.0, 0, 0));
	if (keys[GLFW_KEY_X])
	{
		shouldStop = true;
	}
}