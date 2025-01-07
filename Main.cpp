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
	glfwInit();
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

	WindowData data = { width, height, &camera};
	glfwSetWindowUserPointer(window, &data);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// initialize renderer
	RenderSet Renderer(window, camera, width, height);
	Renderer.set();

	// set skybox and time float
	bool skyboxOn = true;
	int dtRange[31] = { -10000000, -1000000, -100000, -10000, -1000, -500, -200, -100, -64, -32, -16, -8, -4, -2, -1, 0, 
		1, 2, 4, 8, 16, 32, 64, 100, 200, 500, 1000, 10000, 100000, 1000000, 10000000}; // fixed time warp range -10mil to 10mil
	int dt = 16; // current time warp index 
	bool dtChange = false;
	float clickPTime = glfwGetTime();

	Sys.SystemTime();

	GUIData guiData = { Sys.simTime.timeString, dtRange[dt], &dt, Sys.bodies, &(camera.speed), &(camera.focusMode), &(camera.focusBody) };
	GUI gui(window, guiData);

	std::cout << "beginning sim" << std::endl; 
	while (!glfwWindowShouldClose(window)) {
		// constantly checks current state of window 
		glfwPollEvents();
		// process GUI
		gui.guiLoopStart(guiData);

		if (!skyboxOn) {
			glClearColor(0.24f, 0.28f, 0.45f, 1.0f);
		}
		else {
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // need to modify in sys fxn orbHandle
		}
		float clickCTime = glfwGetTime();

		Sys.updateBodyState();

		// input handling -  should add input for safe obliteration (cease while loop)
		// for holding inputs
		(camera).smoothInputs(window, Sys.bodyPos);
		// for single click inputs - must change dt bounds here if dtRange size changes
		(camera).hardInputs(window, Sys.bodyPos, Sys.bodyRadii, skyboxOn, dt);
		// values to be able to see the sun from neptune and handle resized windows
		(camera).updateWindowSize(data.width, data.height);
		(camera).updateMatrix(45.0f, 0.001f, 6100.0f * 2);

		(Renderer).updateWindowSize(data.width, data.height);


		// Process time 
		Sys.WarpClockSet(dtRange[dt]);

		//Render scene
		Renderer.ShadowRender(Sys.bodies, &camera);
		Renderer.Move(Sys.bodies, Sys.lightBodies, Sys.simTime.time_in_sec, dt, (camera).Position);
		Sys.orbLineHandle((camera).Position);
		if (skyboxOn) {
			Renderer.RenderSkyBox(&camera);
		}

		// Render GUI
		gui.guiLoopEnd();

		// update gui info
		guiData.time = Sys.simTime.timeString;
		guiData.tWRange = dtRange[dt];
		guiData.tW = &dt;
		
		//update image each frame
		glfwSwapBuffers(window);

	}

	// terminatation handling
	Sys.deleteSystem(); // need new fxn NOW FROM SYSTEM
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

