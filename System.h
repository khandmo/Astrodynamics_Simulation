#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Mesh.h"
#include "Object.h"  
#include "Textures.h"
#include <vector>

class System {
public:
	int numBodies = 0;
	int numArtBodies = 0;

	std::vector <Mesh> bodiesActual; // holds the body meshes
	std::vector <Mesh*> bodies; // holds the addresses to the body meshes
	std::vector <Mesh*> lightBodies;
	std::vector <Mesh*> dullBodies;
	std::vector<glm::vec3*> bodyPos; // for camera interface

	Shader dS = Shader("default.vert", "default.frag");
	Shader lS = Shader("light.vert", "light.frag");
	Shader* dullShader = &dS;
	Shader* lightShader = &lS;

	// intializer
	System(); // initializes main bodies, shaders / saved game?

	// generates body given, shaderType toggles emission/dull shader modes, adds bodies to pertenant lists
	Mesh initBody(const char* name, const char* texFilePath, glm::vec3 pos, glm::vec3 vel, float mass, float radius, float axialTilt, float angleOfRot, bool isLight, bool areRings, int soiID);

	void updateBodyState(); // handles input during application run-time

	void shaderSet(); // should use list of all emission bodies and list of all diffuse bodies

	void deleteBody(); // deletes bodies that aren't natural satellites

	void deleteShaders(); // deletes shaders for EOP
};


#endif 
 