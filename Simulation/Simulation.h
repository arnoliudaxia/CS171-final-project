
#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include "Body.h"
#include "Collider.h"
#include "Contact.h"
#include "PositionJoint.h"
#include "MouseJoint.h"
#include "PlaneConstraint.h"
#include "RevoluteJoint.h"

class Poly;

class Simulation
{
protected:
	// window dimensions
	int width;
	int height;

	// camera transformation
	float yaw;
	float pitch;

	// mouse co-ordinates
	float mouseX;
	float mouseY;

	// for panning the scene on corner scroll
	bool panLeft, panRight, panTop, panBot;

	// to check for first mouse callback
	bool firstMouseCB;

	// store the state of pressed keys
	bool keys[1024];

	bool stepContinous;
	bool stepOneFrame;

	std::vector<Body> bodies;
	std::vector<Collider*> colliders;
	std::vector<Manifold> manifolds;

	bool picked;

	Model* sphere;

public:
	Simulation();

	void AddObjToScene(const std::string& file, glm::vec3 position, glm::quat orientation, float mass, glm::vec3 color, float restitution = 0.3f, float scale = 1.0f);

	~Simulation() = default;

	virtual void OnInit(GLFWwindow* window);

	void OnWindowResize(GLFWwindow* window, int width, int height);

	void OnMouseMove(GLFWwindow* window, double x, double y);

	virtual void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

	void Step(const float dt);

	virtual void Update();
};