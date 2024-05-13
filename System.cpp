#include "System.h"
System::System() {

	// initialize all solar system bodies, a body's gravitational source must be initialized before that body

	bodiesActual.push_back(initBody("sun", "Textures/SQmercury.jpg", glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), 10.0f, 1.0f, 0.0f, 0.0f, 0.0f, true, false, -1)); // CODE FIX FOR soiID if it is -1 to orbit the center of the universe or stay still
	bodiesActual.push_back(initBody("mercury", "Textures/SQmercury.jpg", glm::vec3(61.4f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.429f), 0.00001651f, 0.0351f, 0.0f, 2, 0.0600068844f, false, false, 0));
	bodiesActual.push_back(initBody("venus", "Textures/SQvenus.jpg", glm::vec3(143.6f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.2641f), 0.00002447f, 0.0869f, 0.0f, 3, 0.0434617764f, false, false, 0));
	bodiesActual.push_back(initBody("earth", "Textures/earth4096.jpg", glm::vec3(200.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.2238f), 0.000030027f, 0.0916f, 0.0f, 23.5, 10.56121166f, false, false, 0));
	bodiesActual.push_back(initBody("moon", "Textures/moon4096.jpg", glm::vec3(205.229f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0010275268), 0.00004f, 0.0249f, 0.0f, 1.5, 0.35800717f, false, false, 3));
	bodiesActual.push_back(initBody("mars", "Textures/SQmars.jpg", glm::vec3(276.28f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.190251f), 0.00003213f, 0.0487f, 0.0f, 25, 10.57f, false, false, 0));
	bodiesActual.push_back(initBody("jupiter", "Textures/SQjupiter.jpg", glm::vec3(990.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.1004081f), 0.009546f, 1.0276f, 0.0f, 3, 25.64f, false, false, 0));
	bodiesActual.push_back(initBody("saturn", "Textures/SQsaturn.jpg", glm::vec3(1808.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0743493f), 0.002857f, 0.8370f, 0.0f, 26.73, 23.6886f, false, false, 0));
	bodiesActual.push_back(initBody("saturnRings", "Textures/SQsaturnRings.jpg", glm::vec3(1808.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0743493f), 0.002857f, 0.93416f, 1.94745f, 26.73, 21.0f, false, true, 0));
	bodiesActual.push_back(initBody("uranus", "Textures/SQuranus.jpg", glm::vec3(3680.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0521264f), 0.0004365f, 0.3645f, 0.0f, 97.7, 14.6939f, false, false, 0));
	bodiesActual.push_back(initBody("neptune", "Textures/SQneptune.jpg", glm::vec3(5962.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.04095276f), 0.0005149f, 0.3539f, 0.0f, 28, 15.8418f, false, false, 0));
	
	// transplant bodies addresses
	for (int i = 0; i < bodiesActual.size(); i++) {
		bodies.push_back(&bodiesActual[i]);
		if (bodies[i]->isLightSource) {
			lightBodies.push_back(bodies[i]);
		}
		else {
			dullBodies.push_back(bodies[i]);
		}
		if (bodies[i]->soiID != -1) {
			bodies[i]->gravSource = &bodiesActual[bodies[i]->soiID];
		}
	}

	// set shaders
	shaderSet();
	updateBodyState(); // shaders stable here
}

Mesh System::initBody(const char* name, const char* texFilePath, glm::vec3 pos, glm::vec3 vel, float mass, float radius, float outerRadius, float axialTilt, float angleOfRot,  bool isLight, bool areRings, int soiID) {
	
	// init texture
	Texture tex[] = { Texture(texFilePath, "diffuse", 0, GL_RGB, GL_UNSIGNED_BYTE) };

	// init object
	Object obj;
	if (!areRings) {
		obj.Sphere(radius, 0.2f, 0.4f, 0.3f); // initialize as sphere, 3 other floats don't matter
	}
	else {
		obj.Rings(radius, outerRadius, 0.2f, 0.4f, 0.3f);
	}
	std::vector <Texture> objTex(tex, tex + sizeof(tex) / sizeof(Texture));
	glm::vec4 lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f); // default color, dummy for dull bodies

	Shader shader = dS;
	if (isLight) {
		shader = lS;
	}

	Mesh body(name, obj.vertices, obj.indices, objTex, isLight, areRings, lightColor, pos, &shader, mass); // velocity will be updated with SPICE integration

	// Set Properties
	body.soiID = soiID;
	body.AxialTilt(axialTilt);
	body.Vel = vel; // assign body velocity
	body.radRot = angleOfRot; // assign rotation speed
	bodyRadii.push_back(radius);

	return body;
}

void System::updateBodyState() { // System holds it's own positions, gets updated from Mesh --- System triggers Move and calls for velocity from body
	// update vector of bodyPos in glm vec3 format
	for (int i = 0; i < bodies.size(); i++) {
		//std::cout << bodies[i]->name << " update body state shader id " << bodies[i]->ShaderProgram->ID << '\n';
		if (bodyPos.size() >= i + 1)
			bodyPos[i] = &(bodies[i]->Pos);
		else
			bodyPos.push_back(&(bodies[i]->Pos));
	}
}

void System::shaderSet() {
	for (auto body : lightBodies) { // sets emissions for all light bodies
		body->emissionShader();
	}
	for (auto body : dullBodies) { // sets shaders for all dull bodies in relation to light bodies
		for (auto lights : lightBodies) {
			body->dullShader(*lights);
		}
	}
}
