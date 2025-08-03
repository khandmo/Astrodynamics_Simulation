#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>
#include "System.h"
#include "Object.h"
#include "Render.h"
#include "GUI.h"


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

struct WindowData {
	int width;
	int height;
	Camera* camera;
};


int main() {
	// glfw initiatizer
	glfwInitHint(GLFW_JOYSTICK_HAT_BUTTONS, GLFW_FALSE);
	if (!glfwInit()) {
		return -1;
	}

	// prominent memory leak breakpoint sets
	//_crtBreakAlloc = 998;
	//_crtBreakAlloc = 1186;
	//_crtBreakAlloc = 1340;
	//_crtBreakAlloc = 3062;
	//_crtBreakAlloc = 3065;
	//_crtBreakAlloc = 3099;
	//_crtBreakAlloc = 3100;
	//_crtBreakAlloc = 420394;
	//_crtBreakAlloc = 240398;
	//_crtBreakAlloc = 651929;
	//_crtBreakAlloc = 651964;
	//_crtBreakAlloc = 653047;
	//_crtBreakAlloc = 653379;
	//_crtBreakAlloc = 2614835;
	//_CrtSetBreakAlloc(1160);


	// Tells glfw what type of opengl I use
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// Tell GLFW we are using the CORE profile
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	int width = 1200; int height = 1200;
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
	Sys.camera = &camera;

	WindowData data = { width, height, &camera};
	glfwSetWindowUserPointer(window, &data);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// initialize renderer
	RenderSet Renderer(window, camera, width, height);
	Renderer.set();

	// set skybox and time float
	bool skyboxOn = true;
	int dtRange[31] = { -1000000, -500000, -100000, -50000, -20000, -10000, -5000, -2000, -1000, -500, -200, -100, -50, -10, -1, 0, 
		1, 10, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 500000, 1000000}; // fixed time warp range -1mil to 1mil
	int dt = 16; // current time warp index 
	bool dtChange = false;
	float clickPTime = glfwGetTime();

	Sys.SystemTime();

	GUIData guiData = { Sys.simTime.timeString, dtRange[dt], &dt, &Sys.simTime.time_in_sec, &Sys };
	GUI gui(window, guiData);

	bool gathered = false;
	std::chrono::time_point<std::chrono::system_clock> prev;
	double prev2;
	_CrtDumpMemoryLeaks();

	std::cout << "beginning sim" << std::endl; 
	while (!glfwWindowShouldClose(window)) {
		// process GUI
		gui.guiLoopStart(guiData);

		if (!skyboxOn) {
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		}
		else {
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // need to modify in sys fxn orbHandle
		}
		float clickCTime = glfwGetTime();



		// input handling -  should add input for safe obliteration (cease while loop)
		// for holding inputs
		(camera).smoothInputs(window, Sys.bodyPos, &Sys.satPos);
		// for single click inputs - must change dt bounds here if dtRange size changes
		(camera).hardInputs(window, Sys.bodyPos, Sys.bodyRadii, &Sys.satPos, skyboxOn, dt);
		// values to be able to see the sun from neptune and handle resized windows
		(camera).updateWindowSize(data.width, data.height);
		(camera).updateMatrix(45.0f, 0.001f, 500000.0f);

		(Renderer).updateWindowSize(data.width, data.height);


		// Process time 
		Sys.WarpClockSet(dtRange[dt]);
		if (gui.goToManTime != -1) {
			Sys.ArgClockSet(gui.goToManTime);
			gui.goToManTime = -1;
		}

		//Render scene
		Renderer.ShadowRender(Sys.bodies, &camera);
		Renderer.Move(Sys.bodies, Sys.lightBodies, Sys.simTime.time_in_sec, dt, (camera).Position, gui.orbitShow);
		

		//Handle changes
		Sys.updateBodyState();
		Sys.orbLineHandle((camera).Position);
		Sys.ArtSatHandle(&camera, Sys.simTime.time_in_sec, dt);


		if (skyboxOn) {
			Renderer.RenderSkyBox(&camera);
		}

		// Render GUI
		gui.guiLoopEnd(guiData);

		// update gui info
		guiData.time = Sys.simTime.timeString;
		guiData.tWRange = dtRange[dt];

		
		//update image each frame
		glfwSwapBuffers(window);
		// constantly checks current state of window 
		glfwPollEvents();

	}

	// terminatation handling
	Sys.~System(); // need new fxn NOW FROM SYSTEM
	glfwDestroyWindow(window);
	glfwTerminate();
	gui.guiDestroy();
	return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	WindowData* data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
	data->width = width;
	data->height = height;
	glViewport(0, 0, data->width, data->height);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	ImGuiIO& io = ImGui::GetIO();
	if (!io.WantCaptureMouse) {
		WindowData* data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
		if (data->camera->focusMode) {
			// focus mode zoom in / zoom out
			if (yoffset < 0) { // zoom in
				data->camera->focusPos *= 1.1;
					//
					// camera->focusPos *= (camera->focusDistSeg[camera->focusDistMarker - 1] / camera->focusDistSeg[camera->focusDistMarker--]);
			}
			if (yoffset > 0) { // zoom out
				data->camera->focusPos /= 1.1;
					//camera->focusPos *= (camera->focusDistSeg[camera->focusDistMarker + 1] / camera->focusDistSeg[camera->focusDistMarker++]);
			}
		}
	}
}

