#include "System.h"
System::System(Shader* dullShaderRcv, Shader* lightShaderRcv) {
	dullShader = dullShaderRcv;
	lightShader = lightShaderRcv;

	// initialize all solar system bodies
	initBody("sun", "Textures/SQmercury.jpg", glm::vec3(0.0f, 0.0f, 0.0f), 10.0f, 10.0f, 0, true, false);
	initBody("mercury", "Textures/SQmercury.jpg", glm::vec3(61.4f, 0.0f, 0.0f), 0.00001651f, 0.0351f, 2, false, false);
	initBody("venus", "Textures/SQvenus.jpg", glm::vec3(143.6f, 0.0f, 0.0f), 0.00002447f, 0.0869f, 3, false, false);
	initBody("earth", "Textures/SQearth.jpg", glm::vec3(200.0f, 0.0f, 0.0f), 0.000030027f, 0.0916f, 23.5, false, false);
	initBody("moon", "Textures/SQmoon.jpg", glm::vec3(205.229f, 0.0f, 0.0f), 0.00004f, 0.0249f, 1.5, false, false);
	initBody("mars", "Textures/SQmars.jpg", glm::vec3(276.28f, 0.0f, 0.0f), 0.00003213f, 0.0487f, 25, false, false);
	initBody("jupiter", "Textures/SQjupiter.jpg", glm::vec3(990.0f, 0.0f, 0.0f), 0.009546f, 1.0276f, 3, false, false);
	initBody("saturn", "Textures/SQsaturn.jpg", glm::vec3(1808.0f, 0.0f, 0.0f), 0.002857f, 0.8370f, 26.73, false, false);
	initBody("saturnRings", "Textures/SQsaturnRings.jpg", glm::vec3(1808.0f, 0.0f, 0.0f), 0.002857f, 0.93416f, 26.73, false, true);
	initBody("uranus", "Textures/SQuranus.jpg", glm::vec3(3680.0f, 0.0f, 0.0f), 0.0004365f, 0.3645f, 97.7, false, false);
	initBody("neptune", "Textures/SQneptune.jpg", glm::vec3(5962.0f, 0.0f, 0.0f), 0.0005149f, 0.3539f, 28, false, false);

	// set shaders
	shaderSet();
	updateBodyPos();
}

void System::initBody(const char* name, const char* texFilePath, glm::vec3 pos, float mass, float radius, float axialTilt, bool isLight, bool areRings) {
	// init texture
	Texture tex[] = {
		Texture(texFilePath, "diffuse", 0, GL_RGB, GL_UNSIGNED_BYTE)
	};

	// init object
	Object obj;
	obj.Sphere(radius, 0.2f, 0.4f, 0.3f); // initialize as sphere, 3 other floats don't matter
	std::vector <Texture> objTex(tex, tex + sizeof(tex) / sizeof(Texture));
	glm::vec4 lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f); // default color, dummy for dull bodies

	Shader shader = *lightShader;
	if (!isLight) {
		shader = *dullShader; // Initialize dull shader program
	}
	

	Mesh body(name, obj.vertices, obj.indices, objTex, isLight, areRings, lightColor, pos, &shader, mass); // velocity will be updated with SPICE integration
	body.AxialTilt(axialTilt);

	bodies.push_back(&body);
	if (isLight) {
		lightBodies.push_back(&body); // add to main list of bodies
	}
	else {
		dullBodies.push_back(&body);
	}
}

void System::updateBodyPos() {
	// update vector of bodyPos in glm vec3 format
	for (int i = 0; i < bodies.size(); i++) {
		if (bodyPos.size() > i + 1)
			bodyPos[i] = &(bodies[i]->Pos);
		else
			bodyPos.push_back(&(bodies[i]->Pos));
	}

}

void System::shaderSet() {
	for (auto body : lightBodies) { // sets emissions for all light bodies
		(*body).emissionShader();
	}
	for (auto body : dullBodies) { // sets shaders for all dull bodies in relation to light bodies
		for (auto lights : lightBodies) {
			(*body).dullShader(*lights);
		}
	}
}
