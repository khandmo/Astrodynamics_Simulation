#include "Render.h"

unsigned int loadCubeMap(std::vector<const char*> faces);

RenderSet::RenderSet(GLFWwindow* window, Camera& camera, int width, int height) {
	RenderSet::window = window;
	RenderSet::camera = &camera;
	screenWidth = width;
	screenHeight = height;

	// create framebuffer
	glGenFramebuffers(1, &depthMapFBO);
	glGenTextures(1, &depthMap);
}


void RenderSet::set() {
	// generate 2D depth map
	
	glBindTexture(GL_TEXTURE_2D, depthMap);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadowWidth,
		shadowHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// attach to framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
		depthMap, 0);
	// do not render color data
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	// generate skybox
	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// load skybox textures
	std::vector<const char*> faces{
		"Textures/px.png",
		"Textures/nx.png",
		"Textures/py.png",
		"Textures/ny.png",
		"Textures/pz.png",
		"Textures/nz.png"
	};
	cubemapTex = loadCubeMap(faces);
	// send uniforms to shader
	skyboxShader.Activate();
	glUniform1i(glGetUniformLocation(skyboxShader.ID, "skybox"), 0);

	// SHADOW DEBUGGING TOOLS
	//debug.Activate();
	//glUniform1i(glGetUniformLocation(debug.ID, "depthMap"), 0);

}

void RenderSet::ShadowRender(std::vector<Mesh*> &bodies, Camera* camera) {
	// activate shadow shader with above matrix
	shadowProgram.Activate();

	// set light shader matrix
// objects not in depth map will not produce shadows
	float near_plane = 1.0f, far_plane = 17.5f; // if lightView matrix focuses on planets, FOV must be wide enough to catch moon(s)
	glm::mat4 lightProjection = glm::ortho(-10.0f, 10.0f,
		-10.0f, 10.0f, near_plane, far_plane);

	int closestBody = 0;
	float distanceToClosest = 10000;
	// find closest planet to camera (body 0 is the sun and bodies 4 and 8 are not planets)
	for (int i = 1; i < bodies.size(); i++) {
		if (i != 4 && i != 8) {
			float bodyDist = abs(glm::length(camera->Position - bodies[i]->Pos));
			if (distanceToClosest > bodyDist) { // vector to body
				distanceToClosest = bodyDist;
				closestBody = i;
			}
		}
	}

	glm::mat4 lightView = glm::lookAt( // has be based on focused body position if a focused body exists
		bodies[closestBody]->Pos - (7.0f * glm::normalize(bodies[closestBody]->Pos)), // light source position - a fixed distance from the closest planet to camera
		bodies[closestBody]->Pos, // position looking towards - closest body position
		glm::vec3(0.0f, 1.0f, 0.0f));

	lightSpaceMatrix = lightProjection * lightView;

	// render to depth map
	glViewport(0, 0, shadowWidth, shadowHeight);
	glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// render scene with shadow shader
	for (int i = 0; i < bodies.size(); i++) {
		bodies[i]->setShadowShader(shadowProgram, lightSpaceMatrix);
		bodies[i]->switchShader();
		if (i != 0) { // if not sun
			bodies[i]->dullShader(*bodies[0]); // send uniforms to depth shaders
			bodies[i]->Draw(*camera); // draw body (MUST BE CONSECUTIVE)
		}
	}
	//RenderObjects(bodies, numBodies); // draw bodies (lay texture(s) and bind)
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	

	
	// render scene normally with shadow mapping
	glViewport(0, 0, screenWidth, screenHeight);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	for (int i = 0; i < bodies.size(); i++) {
		bodies[i]->switchShader();
		bodies[i]->setDepthMap(depthMap); // give depthMap to Mesh
		if (i != 0) { // if not sun
			bodies[i]->dullShader(*bodies[0]);  // send uniforms to normal shaders
		}
		bodies[i]->Draw(*camera); // draw bodies (lay texture(s) and bind)
	}

	// SHADOW DEBUGGING TOOLS
	/*
	debug.Activate();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthMap);

	unsigned int quadVAO = 0;
	unsigned int quadVBO;
		if (quadVAO == 0)
		{
			float quadVertices[] = {
				// positions        // texture Coords
				-1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
				-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
				 1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
				 1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
			};
			// setup plane VAO
			glGenVertexArrays(1, &quadVAO);
			glGenBuffers(1, &quadVBO);
			glBindVertexArray(quadVAO);
			glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
			glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
		}
		glBindVertexArray(quadVAO);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glBindVertexArray(0);*/
}

void RenderSet::Move(std::vector<Mesh*> &bodies, std::vector<Mesh*> &lBodies, float dt) {
	// dt is time step for orbit calculation - dt > 1 slows time by factor, dt < 1 speeds time by factor

	// Every object that orbits must also have a rotate function, if it should not rotate set the first parameter to 0.0f
	// Object will not be drawn if both functions are not present

	for (int i = 1; i < bodies.size(); i++) {
		for (auto lBody : lBodies) {
			(*bodies[i]).Rotate(lBody, dt);
			(*bodies[i]).Orbit(lBody, dt);
		}
	}	
}

void RenderSet::RenderSkyBox(Camera* camera) {
	glDepthFunc(GL_LEQUAL);
	skyboxShader.Activate();
	glm::mat4 view = glm::mat4(1.0f);
	view = glm::lookAt(glm::vec3(0.0f), (*camera).Orientation, (*camera).Up);
	glUniformMatrix4fv(glGetUniformLocation(skyboxShader.ID, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(skyboxShader.ID, "proj"), 1, GL_FALSE, glm::value_ptr((*camera).proj));
	
	glBindVertexArray(skyboxVAO);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTex);

	glDrawArrays(GL_TRIANGLES, 0, 36);
	glBindVertexArray(0);
	glDepthFunc(GL_LESS);
}

unsigned int loadCubeMap(std::vector<const char*> faces) {
	// loads set of textures to cube map from local path
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
	
	// retreive data and set to cube map
	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++) {
		unsigned char* data = stbi_load(faces[i], &width, &height, &nrChannels, 0);
		if (data) {
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
			stbi_image_free(data);
		}
		else {
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	// wrap textures
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}