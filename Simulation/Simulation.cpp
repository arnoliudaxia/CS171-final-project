
#include "Simulation.h"
#include "Poly.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Camera.h"
#include <memory>
#include "NarrowPhase.h"
#include "Mesh.h"
#include "ObjParser.h"
#include "PrimitiveQuery.h"
#include "Line.h"
#include "BroadPhase.h"
#include "../SystemManager.h"

#define MOUSE_SENSITIVITY 0.1
#define FOV 45.0
#define MAX_BODIES 1000
#define VELOCITY_ITERS 5
#define POSITION_ITERS 3

Simulation::Simulation()
	:firstMouseCB(false), stepContinous(true), stepOneFrame(false), picked(false), sphere(nullptr)
{
	panLeft = panRight = panBot = panTop = false;
}

void Simulation::OnInit(GLFWwindow* window)
{
	glfwGetWindowSize(window, &width, &height);

	mouseX = width / 2.0f;
	mouseY = height / 2.0f;

	glClearColor(0.3f, 0.3f, 0.3f, 0.f);

	// draw the pixel only if the object is closer to the viewer
	glEnable(GL_DEPTH_TEST); // enables depth-testing
	glDepthFunc(GL_LESS);    // interpret smaller values as closer
	Camera::GetInstance().SetProjection(45.0, (float)width / (float)height);
	Camera::GetInstance().SetPosition(glm::vec3(0, 50, 60));


	HMesh mesh;
	ParseObj("resources/box.obj", mesh);
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;

	ModelData model;
	CreateSphere(0.03f, model);
	sphere = new Model(model.vertices, model.indices);

	bodies.reserve(5000);
	colliders.reserve(5000);
	manifolds.reserve(5000);
}

void Simulation::OnWindowResize(GLFWwindow* window, int width, int height)
{
	this->width = width;
	this->height = height;
	glViewport(0, 0, width, height);

	Camera::GetInstance().SetProjection(45.0, (float)width / height);
}

void Simulation::OnMouseMove(GLFWwindow* window, double x, double y)
{
	if (firstMouseCB)
	{
		mouseX = (float)x;
		mouseY = (float)y;
		firstMouseCB = false;
	}

	float dx = mouseX - (float)x;
	float dy = mouseY - (float)y;

	mouseX = (float)x;
	mouseY = (float)y;

	dx *= (float)MOUSE_SENSITIVITY;
	dy *= (float)MOUSE_SENSITIVITY;

	yaw += dx;
	pitch += dy;

	if (mouseX < 20) panLeft = true;
	else panLeft = false;
	if (mouseX > width - 20) panRight = true;
	else panRight = false;
	if (mouseY < 20) panBot = true;
	else panBot = false;
	if (mouseY > height - 20) panTop = true;
	else panTop = false;

	if (pitch > 89.0f) pitch = 89.0f;
	if (pitch < -89.0f) pitch = -89.0f;

	Camera::GetInstance().Rotate(yaw, pitch, 0);

}

void Simulation::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
	}
	float movespeed = 3.0f;
	if (keys[GLFW_KEY_LEFT_SHIFT])
	{
		movespeed = 8.0f;
	}

	if (keys[GLFW_KEY_W])
		Camera::GetInstance().Move(Camera::GetInstance().GetCamZ() * movespeed);
	if (keys[GLFW_KEY_S])
		Camera::GetInstance().Move(-Camera::GetInstance().GetCamZ() * movespeed);
	if (keys[GLFW_KEY_A])
		Camera::GetInstance().Move(-Camera::GetInstance().GetCamX() * movespeed);
	if (keys[GLFW_KEY_D])
		Camera::GetInstance().Move(Camera::GetInstance().GetCamX() * movespeed);
	if (keys[GLFW_KEY_SPACE])
		Camera::GetInstance().Move(Camera::GetInstance().GetCamY() * movespeed);
	if (keys[GLFW_KEY_LEFT_CONTROL])
		Camera::GetInstance().Move(-Camera::GetInstance().GetCamY() * movespeed);

	if (keys[GLFW_KEY_P])
		stepContinous = !stepContinous;
	if (keys[GLFW_KEY_N])
		stepOneFrame = true;
}

void Simulation::Step(const float dt)
{
#pragma omp parallel for schedule(dynamic, 1)
	for (size_t i = 0; i < bodies.size(); i++)
	{
		bodies[i].Update(dt);
	}


	for (auto& minifold : manifolds)
	{
		minifold.contacts.clear();
	}
	manifolds.clear();

	BroadPhase::GetInstance().Update();
	auto& colliderPairs = BroadPhase::GetInstance().ComputePairs();
#pragma omp parallel for schedule(dynamic,1)
	for (auto& pair : colliderPairs)
		DetectCollision(manifolds, pair.first, pair.second);

	for (int i = 0; i < VELOCITY_ITERS; i++)
	{
		for (auto& manifold : manifolds)
		{
			manifold.SolveVelocities();
		}
	}

	for (int i = 0; i < POSITION_ITERS; i++)
	{
		for (auto& manifold : manifolds)
		{
			manifold.SolvePositions();
		}
	}

}

void Simulation::Update()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Physics Update
	float dt = dtG;

	if (!stepContinous)
		Step(dt);
	else
	{
		if (stepOneFrame)
		{
			Step(dt);
			stepOneFrame = false;
		}
	}

	// Graphics update
	static glm::mat4 T(1), R(1), S(1), M(1), VP(1), MVP(1);
	glm::mat4 V = Camera::GetInstance().GetViewMatrix();
	glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
	VP = P * V;
#pragma omp parallel for schedule(dynamic,1)
	for (auto& b : bodies)
	{
		b.Render();
	}

#ifdef gizmos
	BroadPhase::GetInstance().Render();
#endif // gizmos



	for (auto& m : manifolds)
	{
		for (auto& contact : m.contacts)
		{
			T = glm::translate(contact.GetPosition());
			M = T;
			MVP = VP * M;
			sphere->SetMVP(MVP);
			sphere->Render();
		}
	}

	float s = 3.0f;
	//惯性移动（鼠标放在窗口角落）
	if (panLeft)
	{
		yaw += s * (float)MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panRight)
	{
		yaw -= s * (float)MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panTop)
	{
		pitch -= s * (float)MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panBot)
	{
		pitch += s * (float)MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}

	Camera::GetInstance().Update();
}

void Simulation::AddObjToScene(const std::string& file, glm::vec3 position,
	glm::quat orientation, float mass,
	glm::vec3 color, float restitution, float scale)
{
	HMesh mesh;
	ModelData model;
	Collider* collider;
	Body body;
	ParseObj(file, mesh);
	mesh.Scale(glm::vec3(1.f) * scale);
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(position);
	body.SetOrientation(orientation);
	body.SetMass(mass);
	body.SetColor(color);
	body.SetRestitution(restitution);
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

}