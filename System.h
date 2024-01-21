#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Mesh.h"
#include "Object.h"  
#include <vector>

class System {
public:
	int numBodies = 0;
	int numArtBodies = 0;

	std::vector <Mesh*> bodies;
	std::vector <Mesh*> lightBodies;
	std::vector <Mesh*> dullBodies;
	std::vector<glm::vec3*> bodyPos; // for camera interface

	Shader* dullShader;
	Shader* lightShader;


	System(); // initializes main bodies, shaders / saved game?

	// generates body given, shaderType toggles emission/dull shader modes, adds bodies to pertenant lists
	void initBody(const char* name, const char* texFilePath, glm::vec3 pos, float mass, float radius, float axialTilt, bool isLight, bool areRings);

	void updateBodyPos(); // handles input during application run-time

	void shaderSet(); // should use list of all emission bodies and list of all diffuse bodies

	void deleteBody(); // deletes bodies that aren't natural satellites

	void deleteShaders(); // deletes shaders for EOP
};


#endif 

/*
The system will hold all bodies , amount of bodies, names of bodies, camera current body, etc. will control the solar system
Holds emissions bodies in a differnt place, can activate all shaders whenever needed

Maybe give body ID's?
*/
