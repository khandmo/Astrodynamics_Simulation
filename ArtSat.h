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


// needs to be even
#define LINE_BUFF_SIZE_AS 200

struct vertex;
class Mesh;
class Marker;
class Camera;
struct uniform_data;
struct geom_shader_lines_device;

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

struct poi { // point of interest
	glm::dvec3 Pos;
	double time;
};

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

struct maneuver { 
	pvUnit origState;
	pvUnit newState;
	double time;
	const char* name;
	const char* desc;
};


class ArtSat {
public:

	ArtSat(const ArtSat& other) { 
		// operator for making independent copies of an art sat
		name = new char(strlen(other.name) + 1);
		name = other.name;
		for (int i = 0; i < LINE_BUFF_SIZE_AS; i++) {
			lineBuff[i] = other.lineBuff[i];
			relLB[i] = other.relLB[i];
			if (i < (LINE_BUFF_SIZE_AS / 2))
				lBTime[i] = other.lBTime[i];
		}
		lineBuffSize = other.lineBuffSize;
		lB_size_actual = other.lB_size_actual;
		inTime = other.inTime;
		stateTime = other.stateTime;
		simPos = other.simPos;
		soiIdx = other.soiIdx;
		maneuvers = other.maneuvers;
		closeApproachDist = other.closeApproachDist;
		lastManIdx = other.lastManIdx;
		lastEphTime = other.lastEphTime;
		sysThread = other.sysThread;
		threadStop = other.threadStop;
		
		pathDevice = geom_shdr_lines_init_device();
		state = new pvUnit(*other.state);
		stateButChanged = new pvUnit(*other.stateButChanged);
		apoapsis = new poi(*other.apoapsis);
		periapsis = new poi(*other.periapsis);
		stat = new stats(*other.stat);
		prevPV = new pvUnit(*other.prevPV);
		mk1 = nullptr;
		mk2 = nullptr;
	}

	const char* name = nullptr;
	pvUnit lineBuff[LINE_BUFF_SIZE_AS];
	vertex_t relLB[LINE_BUFF_SIZE_AS];
	double lBTime[LINE_BUFF_SIZE_AS / 2];
	int lineBuffSize = -1;
	geom_shader_lines_device_t* pathDevice = nullptr;
	int lB_size_actual = -1;
	bool inTime = true;
	
	std::vector <pvUnit>* dynBuff = nullptr;
	std::vector <double>* dynTimes = nullptr;

	pvUnit* state = nullptr;
	pvUnit* stateButChanged = nullptr;
	double stateTime;
	glm::vec3 simPos = { 0, 0, 0 };

	int soiIdx;
	poi* apoapsis = nullptr;
	poi* periapsis = nullptr;
	stats* stat = nullptr;
	std::vector<maneuver> maneuvers;
	double closeApproachDist;

	Marker* satVis = nullptr;
	Marker* mk1 = nullptr;
	Marker* mk2 = nullptr;

	int lastManIdx = 0;
	double lastEphTime = -1;

	std::future<void> *sysThread = nullptr;
	std::atomic<bool> *threadStop = nullptr;
	bool fxnStop = false;
	bool isCopy = false;
	
	pvUnit* prevPV = nullptr;

	glm::vec4 lineColor = glm::vec4(0, 0, 1, 1.0f);
	float lineWidth = 2;

	ArtSat();

	int ArtSatPlan(pvUnit pv, double dt, int soiID, std::vector <Mesh*> bodies);

	void ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, std::atomic<bool> &stop, double dt, const char name[30], const char desc[30]);

	void ArtSatUpdState(std::vector <Mesh*> bodies, double dt, int tW, double mod);

	void ArtSatRender(Camera* camera, Mesh lightSource);

	~ArtSat();

	void chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt);

	void fillBuff(std::vector<pvUnit>* dynBuff, std::vector<double>* dynTime);

	void chartApproach(std::vector<Mesh*> bodies, int targetID);

	void refreshTraj(std::vector<Mesh*> bodies, double dt);

	void solveManeuver(std::vector<Mesh*> bodies, char* name, pvUnit pv1, pvUnit pv2, double t1, double t2);

};

#endif ART_SAT_H