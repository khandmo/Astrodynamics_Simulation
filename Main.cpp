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

	// initialize solar system, timers, and shaders
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

	/* Buffer holds 3 digit weekday, month, date, hour:minute:second, year in system time
	Can be adjusted for to UTC, but have to know personal time zone.Could put it in as input before simulation begins
	*/




	// set skybox and time float
	bool skyboxOn = true;
	int dtRange[31] = { -10000000, -1000000, -100000, -10000, -1000, -500, -200, -100, -64, -32, -16, -8, -4, -2, -1, 0, 
		1, 2, 4, 8, 16, 32, 64, 100, 200, 500, 1000, 10000, 100000, 1000000, 10000000}; // fixed time warp range -10mil to 10mil
	int dt = 16; // current time warp index
	bool dtChange = false;
	float clickPTime = glfwGetTime();
	Sys.SystemTime();
	std::cout << "beginning sim" << std::endl;

	// begin simulation
	while (!glfwWindowShouldClose(window)) {
		glClearColor(0.24f, 0.28f, 0.45f, 1.0f);
		float clickCTime = glfwGetTime();

		Sys.updateBodyState();

		// input handling -  should add input for safe obliteration (cease while loop)
		// for holding inputs
		(camera).smoothInputs(window, Sys.bodyPos);
		// for single click inputs - must change dt bounds here if dtRange size changes
		(camera).hardInputs(window, Sys.bodyPos, Sys.bodyRadii, skyboxOn, dt);
		
		(camera).updateMatrix(45.0f, 0.1f, 6100.0f); // values to be able to see the sun from neptune

		// Process time 
		Sys.WarpClockSet(dtRange[dt]);

		//Render scene
		Renderer.ShadowRender(Sys.bodies, &camera);
		//Renderer.Move(Sys.bodies, Sys.lightBodies, dtRange[dt]); // should replace dtRange with simTime variable
		if (skyboxOn) {
			Renderer.RenderSkyBox(&camera);
		}

		//update image each frame
		glfwSwapBuffers(window);
		// constantly checks current state of window 
		glfwPollEvents();
	}

	// terminatation handling
	Sys.deleteSystem(); // need new fxn NOW FROM SYSTEM
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}