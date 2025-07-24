#pragma once
#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Mesh.h"
#include "Object.h"  
#include "Textures.h"
#include "ArtSat.h"

#include <vector>
#include <chrono>
#include <ctime>
#include <future>
#include <mutex>
#include <string.h>



class ArtSat;
class Mesh;
class Camera;

struct state {
	double time;
	char* date;
	glm::vec3 pos;
	glm::vec3 vel;
	int soi;
};

class System {
public:

	Object* obj_sphere = nullptr;
	Object* obj_rings = nullptr;

	std::vector <Mesh> bodiesActual; // holds the body meshes
	std::vector <Mesh*> bodies; // holds the addresses to the body meshes
	std::vector <Mesh*> lightBodies;
	std::vector <Mesh*> dullBodies;
	std::vector<glm::vec3*> bodyPos; // for camera interface
	std::vector <float> bodyRadii; // for camera

	Camera* camera;
	std::vector<ArtSat> artSats;
	std::vector<glm::vec3*> satPos; // for camera

	// async threads
	std::future<void> maneuverThread;
	std::future<void> twThread;
	std::mutex mtx;
	std::mutex mtxT;
	std::mutex mtxSat;

	std::atomic<bool> satManStop = false;


	Shader dS = Shader("default.vert", "default.frag");
	Shader lS = Shader("light.vert", "light.frag");
	Shader* dullShader = &dS;
	Shader* lightShader = &lS;

	struct time_block {
		std::chrono::time_point<std::chrono::system_clock> timeUnit; // holds the system clock 
		char timeString[30]; // holds the string representation - DDD MMM DD HH:MM:SS YYYY
		int ms = 0; // holds three digit fractional amount
		double time_in_sec = 0; // holds clock time in seconds since UNIX epoch - Jan 1 1970
	};

	time_block sysTime;
	time_block simTime;

	const char* persistent_memory_address;

	// intializer
	System(); // initializes main bodies, shaders / saved game?

	// generates body given, shaderType toggles emission/dull shader modes, adds bodies to pertenant lists
	Mesh initBody(const char* name, const char* texFilePath, float mass, float radius, float outerRadius, float axialTilt, bool isLight, bool areRings, const char* soiID, int baryID, int spiceID, float orbPeriod);

	// generates real missions based on ephemeris data
	void initSat(const char* name, const char* eph);

	// loads sats already saved in persistent memory file
	void initPersistSats(const char* file);

	// add user generated sat to persistent memory file
	bool sat2Persist(ArtSat sat, bool add);

	// passive satellite handling
	void ArtSatHandle(Camera* camera, double dt, int &tW);

	void updateBodyState(); // handles input during application run-time

	void orbLineHandle(glm::vec3 cameraPos);

	// return time of last/next hohmann transfer from b1 to b2
	double hohmannCalc(int b1, int b2, double dt, double& synT);

	// determine distance between two bodies at time dt
	double bodyDistTo(int b1, int b2, double dt);

	void shaderSet(); // should use list of all emission bodies and list of all diffuse bodies

	~System(); // deletes shaders and clears memory for EOP



	// Time functions
	// acquires and saves current system time to System variable
	void SystemTime();

	// function to handle 3 digit ms preservation
	void time_block_ms_add(time_block &someTime, int sum, bool sign);

	// sets sim clock appropriate, takes warp speed into account
	void WarpClockSet(int currWarp);

	// set sim clock to specific value
	void ArgClockSet(double dt);
};
#endif 
 