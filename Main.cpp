
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include "Simulation/CollisionDetectionTest.h"
#include "SystemManager.h"

Simulation* sim = &CollisionDetectionTest::GetInstance();

int main()
{
	// Init GLFW
	if (!glfwInit())
	{
		std::cout << "Failed to initialize GLFW" << std::endl;
	}

	// Init window
	////GLFWwindow* window = glfwCreateWindow(800, 600, "Combo", nullptr, nullptr);
	GLFWwindow* window = glfwCreateWindow(1080, 720, "Project", nullptr, nullptr);
	if (window == nullptr)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();

		return 1;
	}
	glfwMakeContextCurrent(window);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	sim->OnInit(window);

	glfwSetWindowSizeCallback(window, OnWindowResize);
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPosCallback(window, OnMouseMove);
	glfwSetKeyCallback(window, OnKeyInput);

	auto fpsClock = clock();
	// Game Loop
	while (!glfwWindowShouldClose(window))
	{
		// register callback events
		glfwPollEvents();

		// Update Systems
		sim->Update();

		glfwSwapBuffers(window);
		dtG = 1.0f / 60.f * 2.f;
		//dtG = (((double)(clock() - fpsClock)) / CLOCKS_PER_SEC) * 2.f;
		std::cout << "FPS: " << 1 / dtG << std::endl;
		fpsClock = clock();
	}

	std::cout << "Window closed" << std::endl;
	glfwTerminate();

	return 0;
}