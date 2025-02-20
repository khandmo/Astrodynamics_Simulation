#pragma once
#ifndef ART_SAT_H
#define ART_SAT_H

#include "Mesh.h" // might not be the right include
#include "Lines/geometry_shader_lines.h"
#include "SpiceUsr.h"
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>


// needs to be even
#define LINE_BUFF_SIZE_AS 200

struct vertex;
class Mesh;
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
	pvUnit deltaV;
	double time;
};


class ArtSat {
public:

	const char* name = nullptr;
	vertex_t lineBuff[LINE_BUFF_SIZE_AS];
	vertex_t relLB[LINE_BUFF_SIZE_AS];
	geom_shader_lines_device_t pathDevice;
	int lB_size_actual = -1;

	Shader sp = Shader("default.vert", "default.frag");
	
	pvUnit* state = new pvUnit;
	pvUnit* stateButChanged = new pvUnit;
	double stateTime;
	glm::vec3 simPos = { 0, 0, 0 };

	Object* sat;
	int soiIdx;
	poi* apoapsis = new poi;
	poi* periapsis = new poi;
	stats* stat = new stats;
	std::vector<maneuver>* maneuvers = new std::vector<maneuver>;
	
	pvUnit* prevPV = nullptr;

	glm::vec4 lineColor = glm::vec4(0, 0, 1, 1.0f);
	float lineWidth = 2;

	glm::mat4 Model;
	glm::mat4 lsMatrix = glm::mat4(1.0f); // light space matrix for shadows
	VAO VAO;

	// will need all orbital parameters to initialize - apoapsis, periapsis, eccentricity, time at apoapsis to get position correct
	// will need a function that can act as a dummy to plan on-the-fly inits visually with sliders
	ArtSat();

	int ArtSatPlan(pvUnit pv, double dt, int soiID, std::vector <Mesh*> bodies);

	void ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, double dt);

	void ArtSatUpdState(std::vector <Mesh*> bodies, double dt);

	void ArtSatRender(Camera* camera, Mesh lightSource);

	~ArtSat();

	void setShader(Mesh& lightSource);

	void chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt);

	void refreshTraj(std::vector<Mesh*> bodies, double dt);

};

#endif ART_SAT_H