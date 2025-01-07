#pragma once
#ifndef MESH_CLASS_H
#define MESH_CLASS_H

#include <string>
#include "VAO.h"
#include "EBO.h"
#include "Camera.h"
#include "Textures.h"
#include "Shaders.h"
#include "SpiceUsr.h"
#include "Lines/geometry_shader_lines.h"

#define UTC2J2000	946684800
#define LARGEST_DISTANCE 4550000000
#define MAX_VERTS 3 * 100 // 3 * 12 * 1024 * 1024
#define LINE_BUFF_SIZE_U 49 // SHOULD ALWAYS BE ODD for "refinedList" logic
#define LINE_BUFF_SIZE (LINE_BUFF_SIZE_U * 2)
#define REF_LIST_SIZE_U 41 // must be odd
#define REF_LIST_SIZE (REF_LIST_SIZE_U * 2)

struct Camera;
/*
Mesh holds all physical data about an object including the model data, whether or not it is a light source or a ring system,
as well as rotation speed, mass and velocity - orbits are calculated and the model is appropriately manipulated here.
*/

// distnace find formulae
double distanceFind(std::vector<double> pt1, std::vector<double> pt2);
double distanceFind(glm::vec3 state1, SpiceDouble* state2);
double distanceFind(glm::vec3 state1, glm::vec3 state2);

class Mesh {
public:
	const char* name;
	std::vector <Vertex> vertices;
	std::vector <GLuint> indices;
	std::vector <Texture> textures;
	GLuint depthMap = 0;
	Mesh* gravSource; // body whose SOI this body is currently in / body which exerts the greatest grav field on body in solar system
	const char* soiID; // sphere of influence identification for 2 body equations and for "isMoon" flag / if soiID != 0
	int spiceID;
	int baryID;
	int orbitalPeriod;

	bool isLightSource;
	bool areRings;
	bool isMoon;
	glm::vec4 Color; // only if object is a light source
	glm::vec3* Pos = nullptr; // world position ********************************** will need to delete these in Mesh destructor
	glm::vec3* oPos = nullptr; // holder for moon relative pos
	glm::mat4 Model;

	geom_shader_lines_device_t pathDevice;
	vertex_t lineBuffer[(LINE_BUFF_SIZE)];
	int lineBufferSize = LINE_BUFF_SIZE;
	glm::vec4 lineColor = glm::vec4(1, 0, 0, 0.5f);
	int lineWidth = 2;
	double lBVertDt = 0;

	vertex_t* refinedList;
	double refListDt;
	int refListStartIdx = 0;
	int bIdx = -1, rIdx = -1;
	double refNodeMarkerTime, lBNodeMarkerTime, bt = 0, rt = 0;
	int refinedRadius = 15;
	int refinedListSize = REF_LIST_SIZE;
	int flipper;
	float refVertsSum = 0;
	double lastItTime = NULL;
	int lastTWIndex = -1;


	bool sign = false; // if rings, checks to see if sun has crossed ring plane

	GLfloat mass;
	GLfloat h = 0; // specific angular momentum 
	GLfloat redMass = 0;
	float ecc = 0;
	float epsilon = 0;

	float radRot;
	GLfloat currAngleRad = 0;
	GLfloat axisTiltDegree = 0;
	glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);

	Shader ShaderProgram = Shader("default.vert", "default.frag"); // set to be modified
	Shader shadowShaderProgram = Shader("default.vert", "default.frag");
	glm::mat4 lsMatrix = glm::mat4(1.0f); // light space matrix for shadows

	VAO VAO;




	Mesh(const char* objName, std::vector<Vertex> vertices, std::vector <GLuint> indices, std::vector <Texture> textures, bool isLight, bool areRings, Shader *shaderProgram, const char* soiID, int baryIDx, int spiceIDx, double UTCtime, int orbPeriod);

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

	// return functions to call for attributes below

	
};



#endif
