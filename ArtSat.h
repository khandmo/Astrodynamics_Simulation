#pragma once
#ifndef ART_SAT_H
#define ART_SAT_H

#include "Mesh.h"
#include "Object.h"
#include "Marker.h"
#include "Lines/geometry_shader_lines.h"
#include "SpiceUsr.h"

#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <future>
#include <thread>
#include <atomic>
#include <stack>


// needs to be even
#define LINE_BUFF_SIZE_AS 500
// yoshida coefficients
#define yw0 (-cbrt(2) / (2 - cbrt(2)))
#define yw1 (1 / (2 - cbrt(2)))
#define tW_thresh 23


struct vertex;
class Mesh;
class Marker;
class Camera;
struct uniform_data;
struct geom_shader_lines_device;

// state data
struct pvUnit {
	glm::dvec3 Pos;
	glm::dvec3 Vel;

	pvUnit operator+(pvUnit obj) {
		pvUnit res;
		res.Pos = Pos + obj.Pos;
		res.Vel = Vel + obj.Vel;
		return res;
	}
	pvUnit operator-(pvUnit obj) {
		pvUnit res;
		res.Pos = Pos - obj.Pos;
		res.Vel = Vel - obj.Vel;
		return res;
	}
	pvUnit operator*(double f) {
		pvUnit res;
		res.Pos = Pos * f;
		res.Vel = Vel * f;
		return res;
	}
	void operator=(pvUnit obj) {
		Pos = obj.Pos;
		Vel = obj.Vel;
	}
	bool operator==(pvUnit obj) {
		if (Pos != obj.Pos)
			return false;
		if (Vel != obj.Vel)
			return false;
		return true;
	}
	bool operator!=(pvUnit obj) {
		if (*this == obj)
			return false;
		else
			return true;
	}
};
// point of interest
struct poi { 
	glm::dvec3 Pos;
	double time;
};
// data set for user statistics
struct stats { // pass to GUI for user display
	double apoapsis = 0;
	double timeToApo = 0;

	double periapsis = 0;
	double timeToPeri = 0;

	double orbitalPeriod = 0;
	double initTime = 0;
	double MET = 0;

	double distToSoi = 0;
};
struct targStats {
	int targetIdx = 0;

	double closeAppr = 0;
	double soiRad = 0;
	double timeToCloseAppr = 0;

	bool soiCapture = false;
	double timeToCapture = 0;
};
// data set for maneuver
struct maneuver {
	pvUnit origState;
	pvUnit newState;
	double time;
	const char* name;
	const char* desc;
	int soi;

	maneuver(pvUnit oS, pvUnit nS, double t, const char* n, const char* d, int s)
		: origState(oS), newState(nS), time(t), name(_strdup(n)), desc(_strdup(d)), soi(s) {}

	maneuver(const maneuver& other) {
		origState = other.origState;
		newState = other.newState;
		time = other.time;
		name = _strdup(other.name);  // allocate new memory
		desc = _strdup(other.desc);
		soi = other.soi;
	}

	~maneuver() {
		free((void*)name);  // cast away const for free
	}

	maneuver& operator=(const maneuver& other) {
		if (this != &other) {
			origState = other.origState;
			newState = other.newState;
			time = other.time;
			free((void*)name);
			name = _strdup(other.name);
			free((void*)desc);
			desc = _strdup(other.desc);
			soi = other.soi;
		}
		return *this;
	}
};

// finds angle between two vectors
float getAngle(glm::vec3 v1, glm::vec3 v2);

class ArtSat {
public:

	ArtSat(const ArtSat& other) { 
		// operator for making independent copies of an art sat
		name = new char(strlen(other.name) + 1);
		name = other.name;

		int fullSize = LINE_BUFF_SIZE_AS * other.lineBuffSect.size();
		// modify buffer sizes if necessary
		if (other.lineBuffSize > LINE_BUFF_SIZE_AS) {
			delete[] lineBuff;
			delete[] relLB;
			delete[] lBTime;
			lineBuff = new pvUnit[fullSize];
			relLB = new vertex_t[fullSize];
			lBTime = new double[fullSize / 2];
		}
		std::copy(other.lineBuff, other.lineBuff + (fullSize), lineBuff);
		std::copy(other.relLB, other.relLB + (fullSize), relLB);
		std::copy(other.lBTime , other.lBTime + (fullSize / 2), lBTime);

		lineBuffSize = other.lineBuffSize;
		lB_size_actual = other.lB_size_actual;
		inTime = other.inTime;
		lineBuffSect = other.lineBuffSect;
		stateTime = other.stateTime;
		simPos = other.simPos;
		soiIdx = other.soiIdx;
		maneuvers = other.maneuvers;
		closeApproachTime = other.closeApproachTime;
		captureTime = other.captureTime;
		lastManIdx = other.lastManIdx;
		lastEphTime = other.lastEphTime;
		sysThread = other.sysThread;
		threadStop = other.threadStop;
		soi_ing = other.soi_ing;
		mtxSat = other.mtxSat;
		
		state = new pvUnit(*other.state);
		stateButChanged = new pvUnit(*other.stateButChanged);
		apoapsis = new poi(*other.apoapsis);
		periapsis = new poi(*other.periapsis);
		stat = new stats(*other.stat);
		if (other.targStat != nullptr)
			targStat = new targStats(*other.targStat);
		prevPV = new pvUnit(*other.prevPV);
		mk1 = nullptr;
		mk2 = nullptr;
	}

	// sat name
	const char* name = nullptr;
	// set of orbital nodes for render and analysiss
	pvUnit* lineBuff = new pvUnit[LINE_BUFF_SIZE_AS];
	// render set of vertex's derived from lineBuff
	vertex_t* relLB = new vertex_t[LINE_BUFF_SIZE_AS];
	// UNIX time for each lineBuff node
	double* lBTime = new double[LINE_BUFF_SIZE_AS / 2];
	// whole size for render
	int lineBuffSize = -1;
	// holds lineBuff index of change and soiIdx of new body
	std::vector<std::pair<int,int>> soiNodes;
	// struct for openGL line rendering
	geom_shader_lines_device_t* pathDevice = geom_shdr_lines_init_device();
	// handles amt of relLB nodes rendered
	int lB_size_actual = -1;
	// true if satellite currently exists at sim time
	bool inTime = true;
	// orbit sections in sequential order with soi, node # pairs
	std::vector<std::pair<int, int>> lineBuffSect;
	std::stack<int> trajTransitions;
	
	// dynamic sets for prelim lineBuff and lBTime
	std::vector <pvUnit>* dynBuff = nullptr;
	std::vector <double>* dynTimes = nullptr;

	// state rel to soiIdx in km
	pvUnit* state = nullptr;
	// state rel to soiIdx in soiChanged units
	pvUnit* stateButChanged = nullptr;
	double stateTime;
	glm::vec3 simPos = { 0, 0, 0 };

	// body list soi index
	int soiIdx;
	poi* apoapsis = nullptr;
	poi* periapsis = nullptr;
	stats* stat = nullptr;
	targStats* targStat = nullptr;
	// set of all satellite maneuver data
	std::vector<maneuver> maneuvers;
	double closeApproachTime;
	double captureTime;
	double lastTargIdx = 0;

	// data for sat marker, close approach markers
	Marker* satVis = nullptr;
	int mk1Soi = -1;
	Marker* mk1 = nullptr;
	Marker* mk2 = nullptr;

	int lastManIdx = 0;
	double lastEphTime = -1;

	// access to system threads and atomic stop for parallel computation
	std::future<void> *sysThread = nullptr;
	std::atomic<bool> *threadStop = nullptr;
	std::atomic<bool> *soi_ing = new std::atomic<bool>(false);
	std::mutex* mtxSat = nullptr;
	bool fxnStop = false;
	bool escaping = false;
	bool isCopy = false;
	bool planning = false;
	
	pvUnit* prevPV = nullptr;

	glm::vec4 lineColor = glm::vec4(0, 0, 1, 1.0f);
	float lineWidth = 2;

	ArtSat();

	int ArtSatPlan(pvUnit pv, double dt, int soiID, std::vector <Mesh*> bodies);

	void ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, double dt, const char name[30], const char desc[30]);

	void ArtSatUpdState(std::vector <Mesh*> bodies, double dt, int &tW, double mod);

	void ArtSatRender(Camera* camera, Mesh lightSource);

	~ArtSat();

	void chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt, int altSoi);

	void fillBuff(int mod);

	void chartApproach(std::vector<Mesh*> bodies, int targetID);

	void refreshTraj(std::vector<Mesh*> bodies, double dt);

	void solveManeuver(std::vector<Mesh*> bodies, char* name, pvUnit pv1, pvUnit pv2, double t1, double t2);

};

#endif ART_SAT_H