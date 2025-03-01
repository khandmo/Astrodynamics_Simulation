#pragma once
#ifndef ART_SAT_H
#define ART_SAT_H

#include "Mesh.h" // might not be the right include
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


// needs to be even
#define LINE_BUFF_SIZE_AS 200

struct vertex;
class Mesh;
class Marker;
class Camera;
class Object;
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

	ArtSat& operator^(const ArtSat& other) { 
		// operator for making independent copies of an art sat
		delete sat;
		delete state;
		delete stateButChanged;
		delete apoapsis;
		delete periapsis;
		delete stat;
		delete prevPV;
		delete mk1;
		delete mk2;
		geom_shader_lines_device_t dummy = pathDevice;
		VAO dumber = VAOu;

		*this = other;
		
		pathDevice = dummy;
		VAOu = dumber;

		sat = new Object(*other.sat);
		state = new pvUnit(*other.state);
		stateButChanged = new pvUnit(*other.stateButChanged);
		apoapsis = new poi(*other.apoapsis);
		periapsis = new poi(*other.periapsis);
		stat = new stats(*other.stat);
		prevPV = new pvUnit(*other.prevPV);
		mk1 = nullptr;
		mk2 = nullptr;
		return *this;
	}

	const char* name = nullptr;
	vertex_t lineBuff[LINE_BUFF_SIZE_AS];
	vertex_t relLB[LINE_BUFF_SIZE_AS];
	geom_shader_lines_device_t pathDevice;
	int lB_size_actual = -1;
	bool inTime = true;

	Shader sp = Shader("default.vert", "default.frag");
	
	pvUnit* state = nullptr;
	pvUnit* stateButChanged = nullptr;
	double stateTime;
	glm::vec3 simPos = { 0, 0, 0 };

	Object* sat = nullptr;
	int soiIdx;
	poi* apoapsis = nullptr;
	poi* periapsis = nullptr;
	stats* stat = nullptr;
	std::vector<maneuver> maneuvers;
	double closeApproachDist;

	Marker* mk1 = nullptr;
	Marker* mk2 = nullptr;

	double lastEphTime = -1;


	
	pvUnit* prevPV = nullptr;

	glm::vec4 lineColor = glm::vec4(0, 0, 1, 1.0f);
	float lineWidth = 2;

	glm::mat4 Model;
	glm::mat4 lsMatrix = glm::mat4(1.0f); // light space matrix for shadows
	VAO VAOu;

	ArtSat();

	int ArtSatPlan(pvUnit pv, double dt, int soiID, std::vector <Mesh*> bodies);

	void ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, double dt, const char name[30], const char desc[30]);

	void ArtSatUpdState(std::vector <Mesh*> bodies, double dt, int tW, double mod);

	void ArtSatRender(Camera* camera, Mesh lightSource);

	~ArtSat();

	void setShader(Mesh& lightSource);

	void chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt);

	void chartApproach(std::vector<Mesh*> bodies, int targetID);

	void refreshTraj(std::vector<Mesh*> bodies, double dt);

};

#endif ART_SAT_H