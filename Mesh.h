#pragma once
#ifndef MESH_CLASS_H
#define MESH_CLASS_H

#include <string>
#include <thread>
#include <mutex>

#include "VAO.h"
#include "EBO.h"
#include "Camera.h"
#include "Textures.h"
#include "Shaders.h"
#include "SpiceUsr.h"
#include <iomanip>
#include "Lines/geometry_shader_lines.h"

#define UTC2J2000	946684800
#define LARGEST_DISTANCE 4550000000
#define LENGTH_SCALE ((float)6100 / LARGEST_DISTANCE)
#define MAX_VERTS 3 * 12 * 1024 * 1024
#define LINE_BUFF_SIZE_U 99 // SHOULD ALWAYS BE ODD for "refinedList" logic
#define LINE_BUFF_SIZE (LINE_BUFF_SIZE_U * 2)
#define REF_LIST_SIZE_U 41 // must be odd
#define REF_LIST_SIZE (REF_LIST_SIZE_U * 2)

struct Camera;
struct pvUnit;
/*
Mesh holds all physical data about an object including the model data, whether or not it is a light source or a ring system,
as well as rotation speed, mass and velocity - orbits are calculated and the model is appropriately manipulated here.
*/

// distnace find formulae
double distanceFind(std::vector<double> pt1, std::vector<double> pt2);
double distanceFind(glm::vec3 state1, SpiceDouble* state2);
double distanceFind(glm::vec3 state1, glm::vec3 state2);
void stateChange(SpiceDouble* state); // changes SpiceDoubule state[0-2] data from real life to simulation environment
void stateChange(glm::dvec3* pos, glm::dvec3* vel);
void invStateChange(glm::dvec3* pos, glm::dvec3* vel);

class Mesh {
public:

	

	const char* name = nullptr;
	std::vector <Vertex> vertices;
	std::vector <GLuint> indices;
	std::vector <Texture> textures;
	GLuint depthMap = 0;
	Mesh* gravSource = nullptr; // body whose SOI this body is currently in / body which exerts the greatest grav field on body in solar system
	const char* soiID; // sphere of influence identification for 2 body equations and for "isMoon" flag / if soiID != 0
	const char* soiIdx;
	int spiceID = -1;
	int baryID = -1;
	int orbitalPeriod = -1;

	bool isLightSource;
	bool areRings;
	bool isMoon;
	glm::vec4 Color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);; // only if object is a light source
	glm::vec3* Pos = nullptr; // world position in sim units
	glm::vec3* oPos = nullptr; // holder for moon relative pos
	glm::mat4 Model;

	geom_shader_lines_device_t* pathDevice = nullptr;
	vertex_t lineBuffer[(LINE_BUFF_SIZE)]; // if i hold pointers to the vertices maybe i can have more rendered
	int lineBufferSize = LINE_BUFF_SIZE;
	glm::vec4 lineColor = glm::vec4(1, 0, 0, 0.5f);
	int lineWidth = 2;
	double lBVertDt = 0;

	vertex_t* refinedList = nullptr;
	double refListDt = 0;
	int refListStartIdx = 0;
	int bIdx = -1, rIdx = -1;
	double refNodeMarkerTime = 0, lBNodeMarkerTime = 0, bt = 0, rt = 0;
	int refinedRadius = 15;
	int refinedListSize = REF_LIST_SIZE;
	int flipper = 0;
	float refVertsSum = 0;
	double lastItTime = NULL;
	int lastTWIndex = -1;

	std::mutex* spiceMtx;

	bool sign = false; // if rings, checks to see if sun has crossed ring plane

	double mass = 0;
	float radius; // radius in sim units
	float realRadius;
	float escapeVel;
	double soiRadius; // radius in km

	float radRot = 0;
	GLfloat currAngleRad = 0;
	GLfloat axisTiltDegree = 0;
	glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);

	Shader ShaderProgram = Shader("default.vert", "default.frag"); // set to be modified
	Shader shadowShaderProgram = Shader("default.vert", "default.frag");
	glm::mat4 lsMatrix = glm::mat4(1.0f); // light space matrix for shadows

	VAO VAO;




	Mesh(const char* objName, std::vector<Vertex> vertices, std::vector <GLuint> indices, std::vector <Texture> textures, float radius, float mass, bool isLight, bool areRings,
		Shader *shaderProgram, const char* soiID, const char* soiIdx, int baryIDx, int spiceIDx, double UTCtime, int orbPeriod);

	// sets shader program for depth map
	void setShadowShader(Shader& program, glm::mat4 lightSpaceMatrix);

	// switches active shader program
	void switchShader();

	// sets depth map
	void setDepthMap(GLuint depthMapInput);

	// calculates emission light if mesh is a light source
	void emissionShader();

	// calculates light reflection onto camera if mesh is not a light source and effected by one
	void dullShader(Mesh& lightSource);

	// draws body on screen
	void Draw(Camera& camera);

	// rotates body at speed on specific axis and assigns light shader appropriately
	void Rotate(Mesh* lightSource, double UTCTime);

	// rotates model along x axis by degree given
	void AxialTilt(GLfloat tiltDeg);

	// calculate orbital position and control relative orbital lines
	void Orbit(Mesh* lightSource, double UTCTime, int timeWarpIndex, glm::vec3 cameraPos);  // MIGHT HAVE TO HOLD VELOCITY AND PARENT SOURCE AS MESH PROPERTY, GET LIGHT SOURCES FROM SYSTEM

	// update model position and orientation
	void updateModel(Mesh& source);
	
	//~Mesh();
	// return functions to call for attributes below

	// returns NASA (km) position/velocity wrt Solar System barycenter at a given time
	pvUnit getPV(double time, bool stateChanged, bool rotated);
	
};



#endif

/*
	// copy constructor
	Mesh(const Mesh& other) {
		*this = other;
		if (pathDevice != nullptr) {
			this->pathDevice = geom_shdr_lines_init_device();
		}
	}

	// move constructor
	Mesh(Mesh&& other) noexcept {
		*this = other;
		other.pathDevice = nullptr;
	}

	// copy assignment
	Mesh& operator=(const Mesh& other){
		if (this != &other) {
			if (pathDevice != nullptr) {
				geom_shdr_lines_term_device((void**)&other.pathDevice);
				this->pathDevice = geom_shdr_lines_init_device();
			}

			delete[] name;

			if (other.name) {
				name = new char[strlen(other.name) + 1];
				strcpy(const_cast<char*>(name), other.name);
			}
			else {
				name = nullptr;
			}

			delete Pos;
			delete oPos;
			delete refinedList;

			if (other.Pos) Pos = new glm::vec3(*other.Pos);
			else Pos = nullptr;

			if (other.oPos) oPos = new glm::vec3(*other.oPos);
			else oPos = nullptr;

			if (other.refinedList) {
				refinedList = new vertex_t[other.refinedListSize];
				for (int i = 0; i < other.refinedListSize; i++) {
					refinedList[i] = other.refinedList[i];
				}
			}
			else {
				refinedList = nullptr;
			}

			vertices = other.vertices;
			indices = other.indices;
			textures = other.textures;

			depthMap = other.depthMap;
			VAO = other.VAO;

			gravSource = other.gravSource;
			soiID = other.soiID;
			spiceID = other.spiceID;
			baryID = other.baryID;
			orbitalPeriod = other.orbitalPeriod;
			isLightSource = other.isLightSource;
			areRings = other.areRings;
			isMoon = other.isMoon;
			Color = other.Color;
			Model = other.Model;

			spiceMtx = other.spiceMtx;

			lsMatrix = other.lsMatrix;

			mass = other.mass;
			radius = other.radius;
			realRadius = other.realRadius;
			escapeVel = other.escapeVel;
			radRot = other.radRot;
			currAngleRad = other.currAngleRad;
			axisTiltDegree = other.axisTiltDegree;
			Up = other.Up;

			for (int i = 0; i < sizeof(lineBuffer) / sizeof(vertex_t); i++) {
				lineBuffer[i] = other.lineBuffer[i];
			}
			lineBufferSize = other.lineBufferSize;
			lineColor = other.lineColor;
			lineWidth = other.lineWidth;
			lBVertDt = other.lBVertDt;

			refListDt = other.refListDt;
			refListStartIdx = other.refListStartIdx;
			bIdx = other.bIdx;
			rIdx = other.rIdx;
			refNodeMarkerTime = other.refNodeMarkerTime;
			lBNodeMarkerTime = other.lBNodeMarkerTime;
			bt = other.bt;
			rt = other.rt;
			refinedRadius = other.refinedRadius;
			refinedListSize = other.refinedListSize;
			flipper = other.flipper;
			refVertsSum = other.refVertsSum;
			lastItTime = other.lastItTime;
			lastTWIndex = other.lastTWIndex;
			sign = other.sign;
		}
		return *this;
	}

	// move assignment
	Mesh& operator=(Mesh&& other) noexcept {
		if (this != &other) {

			delete[] name;

			if (other.name) {
				name = new char[strlen(other.name) + 1];
				strcpy(const_cast<char*>(name), other.name);
			}
			else {
				name = nullptr;
			}

			delete Pos;
			delete oPos;
			delete refinedList;

			Pos = other.Pos;
			oPos = other.oPos;
			refinedList = other.refinedList;

			other.Pos = nullptr;
			other.oPos = nullptr;
			other.refinedList = nullptr;

			vertices = std::move(other.vertices);
			indices = std::move(other.indices);
			textures = std::move(other.textures);

			depthMap = other.depthMap;
			other.depthMap = 0;

			VAO = (other.VAO);
			other.VAO.ID = 0;

			gravSource = other.gravSource;
			soiID = other.soiID;
			spiceID = other.spiceID;
			baryID = other.baryID;
			orbitalPeriod = other.orbitalPeriod;
			isLightSource = other.isLightSource;
			areRings = other.areRings;
			isMoon = other.isMoon;
			Color = std::move(other.Color);
			Model = std::move(other.Model);

			ShaderProgram = other.ShaderProgram;
			shadowShaderProgram = other.shadowShaderProgram;
			other.ShaderProgram.Delete();
			other.shadowShaderProgram.Delete();

			spiceMtx = other.spiceMtx;

			lsMatrix = std::move(other.lsMatrix);

			mass = other.mass;
			radius = other.radius;
			realRadius = other.realRadius;
			escapeVel = other.escapeVel;
			radRot = other.radRot;
			currAngleRad = other.currAngleRad;
			axisTiltDegree = other.axisTiltDegree;
			Up = std::move(other.Up);

			pathDevice = other.pathDevice;
			other.pathDevice = nullptr;

			for (int i = 0; i < sizeof(lineBuffer); i++) {
				lineBuffer[i] = other.lineBuffer[i];
			}
			lineBufferSize = other.lineBufferSize;
			lineColor = other.lineColor;
			lineWidth = other.lineWidth;
			lBVertDt = other.lBVertDt;

			refListDt = other.refListDt;
			refListStartIdx = other.refListStartIdx;
			bIdx = other.bIdx;
			rIdx = other.rIdx;
			refNodeMarkerTime = other.refNodeMarkerTime;
			lBNodeMarkerTime = other.lBNodeMarkerTime;
			bt = other.bt;
			rt = other.rt;
			refinedRadius = other.refinedRadius;
			refinedListSize = other.refinedListSize;
			flipper = other.flipper;
			refVertsSum = other.refVertsSum;
			lastItTime = other.lastItTime;
			lastTWIndex = other.lastTWIndex;
			sign = other.sign;

			other.gravSource = nullptr;
			other.soiID = nullptr;
			other.spiceMtx = nullptr;

		}
		return *this;
	}
	*/