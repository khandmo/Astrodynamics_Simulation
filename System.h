#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Mesh.h"
#include "Object.h"  
#include "Textures.h"
#include <vector>
#include <chrono>
#include <ctime>

class System {
public:

	std::vector <Mesh> bodiesActual; // holds the body meshes
	std::vector <Mesh*> bodies; // holds the addresses to the body meshes
	std::vector <Mesh*> lightBodies;
	std::vector <Mesh*> dullBodies;
	std::vector<glm::vec3*> bodyPos; // for camera interface
	std::vector <float> bodyRadii; // for camera

	Shader dS = Shader("default.vert", "default.frag");
	Shader lS = Shader("light.vert", "light.frag");
	Shader* dullShader = &dS;
	Shader* lightShader = &lS;

	struct time_block {
		std::chrono::time_point<std::chrono::system_clock> timeUnit;
		char timeString[30];
		int ms = 0;
	};

	time_block sysTime;
	time_block simTime;

	// intializer
	System(); // initializes main bodies, shaders / saved game?

	// generates body given, shaderType toggles emission/dull shader modes, adds bodies to pertenant lists
	Mesh initBody(const char* name, const char* texFilePath, glm::vec3 pos, glm::vec3 vel, float mass, float radius, float outerRadius, float axialTilt, float angleOfRot, bool isLight, bool areRings, int soiID);

	void updateBodyState(); // handles input during application run-time

	void shaderSet(); // should use list of all emission bodies and list of all diffuse bodies

	void deleteSystem(); // deletes shaders and clears memory for EOP



	// Time functions
	// acquires and saves current system time to System variable
	void SystemTime();

	// function to handle 3 digit ms preservation
	void time_block_ms_add(time_block &someTime, int sum);

	// sets sim clock appropriate, takes warp speed into account
	void WarpClockSet(int currWarp);
};
#endif 
 