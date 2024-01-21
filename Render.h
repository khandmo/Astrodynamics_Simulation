#ifndef RENDER_H
#define RENDER_H

#include <glad/glad.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include "Mesh.h"
#include "Camera.h"

/*
Render handles generation of shadow maps and skybox. Anything not directly related to meshes or camera is manipulated here. Render
also commands the simultaneous motion of bodies and initially prepares the window for display.
*/

class RenderSet {
public:
	GLFWwindow* window;
	Camera* camera;
	int screenWidth;
	int screenHeight;
	int shadowWidth = 4096;
	int shadowHeight = 4096;

	GLuint depthMapFBO;
	GLuint depthMap;
	glm::mat4 lightSpaceMatrix = glm::mat4(1.0f);
	Shader shadowProgram = Shader("depth.vert", "depth.frag");
	Shader debug = Shader("debug.vert", "debug.frag");

	unsigned int skyboxVAO = 0, skyboxVBO = 0;
	Shader skyboxShader = Shader("skybox.vert", "skybox.frag");
	unsigned int cubemapTex = 0;

	// initializer
	RenderSet(GLFWwindow* window, Camera& camera, int width, int height);
	// renders scene
	void RenderObjects(Mesh* bodies[], const int numBodies);
	// generates shadow maps and skybox
	void set();
	// render shadpw maps
	void ShadowRender(Mesh* bodies[], const int numBodies, Camera* camera);
	// move bodies
	void Move(Mesh* bodies[], float dt);
	// render skybox
	void RenderSkyBox(Camera* camera);
};

#endif
