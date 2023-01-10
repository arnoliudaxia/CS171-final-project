
#pragma once

#include "Simulation.h"

class CollisionDetectionTest : public Simulation
{
public:
	static CollisionDetectionTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	CollisionDetectionTest() {};
};