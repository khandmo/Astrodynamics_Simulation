#include "System.h"
#include "Object.h"
#include "Render.h"

int main() {
	// glfw initiatizer
	glfwInit();
	// Tells glfw what type of opengl I use
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// Tell GLFW we are using the CORE profile
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// could change dynamically
	int width = 1400; int height = 1400;

	// window init takes width, height, name, fullscreen or not,?
	GLFWwindow* window = glfwCreateWindow(width, height, "AstroDynPro", NULL, NULL);
	// Error check
	if (window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	// Load GLAD and configure to OpenGL
	gladLoadGL();
	// Specify the window dimensions
	glViewport(0, 0, width, height);

	// initialize solar system & shaders
	std::cout << "initializing" << '\n';
	System Sys;
	std::cout << "initialized" << '\n';

	// Acheive 3D depth
	glEnable(GL_DEPTH_TEST);
	// initialize Camera with initial position
	Camera camera(width, height, glm::vec3(130.0f, 0.0f, 0.0f));
	
	// initialize renderer
	RenderSet Renderer(window, camera, width, height);
	Renderer.set();
	
	
	// set skybox and time float
	bool skyboxOn = true;
	int dtRange[15] = { 100000, 50000, 25000, 10000, 5000, 2500, 1250, 625, 400, 200, 100, 25, 10, 5, 1 }; // fixed time warp range
	int dt = 10; // current time warp index
	float prevTime = glfwGetTime();
	std::cout << "begining sim" << std::endl;

	// begin simulation
	while (!glfwWindowShouldClose(window)) {
		glClearColor(0.24f, 0.28f, 0.45f, 1.0f);
		float currTime = glfwGetTime();
		
		Sys.updateBodyState();
		// input handling -  should add input for safe obliteratiob n (cease while loop)
		(camera).smoothInputs(window, Sys.bodyPos);
		if (currTime - prevTime > 0.08) {
			prevTime = currTime;
			// for single click inputs
			(camera).hardInputs(window, Sys.bodyPos, Sys.bodyRadii);
			if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) { // toggles skybox
				skyboxOn = !skyboxOn;
			}
			// time warp
			if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS) {
				if (dt < 14)
					dt++;
			}
			else if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS) {
				if (dt > 0)
					dt--; // time step decrease
			}
		}
		(camera).updateMatrix(45.0f, 0.1f, 6100.0f); // to be able to see the sun from neptune

		//Render scene
		Renderer.ShadowRender(Sys.bodies, &camera);
		Renderer.Move(Sys.bodies, Sys.lightBodies, (float)(dtRange[dt]) / 100); // dt itself can rely on larger numbers, higher dtRange means fast time warp with inverse
		if (skyboxOn) {
			Renderer.RenderSkyBox(&camera);
		}


		//update image each frame
		glfwSwapBuffers(window);
		// constantly checks current state of window 
		glfwPollEvents();
	}

	// terminatation handling - need new fxn NOW FROM SYSTEM

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}