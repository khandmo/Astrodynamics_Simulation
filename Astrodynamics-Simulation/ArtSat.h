#pragma once
#ifndef ART_SAT_H
#define ART_SAT_H

#include "Mesh.h" // might not be the right include
#include "Lines/geometry_shader_lines.h"
#include "SpiceUsr.h"
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

// needs to be even
#define LINE_BUFF_SIZE_AS 200

struct vertex;
class Mesh;
class Camera;
struct uniform_data;
struct geom_shader_lines_device;

struct pvUnit {
	glm::vec3 Pos;
	glm::vec3 Vel;

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
	pvUnit operator*(float f) {
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


class ArtSat {
public:

	const char* name = nullptr;
	vertex_t lineBuff[LINE_BUFF_SIZE_AS];
	geom_shader_lines_device_t pathDevice;
	// will need to hold vertices, indices, texture info for cube

	
	
	// init should lock in Pos and Vel (at certain time t0), can compute Acc wrt each body at time t, rk4 to get new pos/vel and do again
	pvUnit* pvSOI = nullptr;
	pvUnit* pvSun = nullptr;
	pvUnit* prevPV = nullptr;

	glm::vec4 lineColor = glm::vec4(0, 0, 1, 1.0f);
	float lineWidth = 2;


	// will need all orbital parameters to initialize - apoapsis, periapsis, eccentricity, time at apoapsis to get position correct
	// will need a function that can act as a dummy to plan on-the-fly inits visually with sliders
	ArtSat();

	int ArtSatPlan(pvUnit pv, double dt, int soiID, std::vector <Mesh*> bodies);

	void ArtSatSave();

	void ArtSatManeuver();

	void ArtSatUpdState();

	void ArtSatRender(Camera* camera);

	void deleteArtSat();

};

#endif ART_SAT_H