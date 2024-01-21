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
	System Sys(Shader dShader = Shader("default.vert", "default.frag"), Shader lShader =  Shader("light.vert", "light.frag")); // MUST INTERACT WITH INS

	
	// Acheive 3D depth
	glEnable(GL_DEPTH_TEST);
	// initialize Camera with initial position
	Camera camera(width, height, glm::vec3(130.0f, 0.0f, 0.0f));
	
	// initialize renderer
	RenderSet Renderer(window, camera, width, height);
	Renderer.set();
	
	
	// set skybox and time float
	bool skyboxOn = true;
	float dt = 1.0f;
	float prevTime = glfwGetTime();

	// begin simulation
	while (!glfwWindowShouldClose(window)) {
		glClearColor(0.24f, 0.28f, 0.45f, 1.0f);
		float currTime = glfwGetTime();
		
		System::updateBodyPos;
		// input handling - must send positions of bodies to camera NOW FROM SYSTEM - MAY NEED A FUNCTION TO GENERATE BODY POSITIONS - should add input for safe obliteration (cease while loop)
		(camera).smoothInputs(window, );
		if (currTime - prevTime > 0.08) {
			prevTime = currTime;
			// for single click inputs
			(camera).hardInputs(window, System::bodyPos);
			if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) { // toggles skybox
				skyboxOn = !skyboxOn;
			}
			// time warp
			if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS) {
				dt /= 2; // time step increase
			}
			else if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS) {
				dt *= 2; // time step decrease
			}
		}
		(camera).updateMatrix(45.0f, 0.1f, 6100.0f); // to be able to see the sun from neptune

		//Render scene - also needs bodies, must change functions to take vector instead of array NOW FROM SYSTEM
		Renderer.ShadowRender(bodies, numBodies, &camera);
		Renderer.Move(bodies, dt);
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