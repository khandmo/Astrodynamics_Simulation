#ifndef MESS_CLASS_H
#define MESS_CLASS_H

#include <string>
#include "VAO.h"
#include "EBO.h"
#include "Camera.h"
#include "Textures.h"
#include "Shaders.h"
#include "SpiceUsr.h"

/*
Mesh holds all physical data about an object including the model data, whether or not it is a light source or a ring system,
as well as rotation speed, mass and velocity - orbits are calculated and the model is appropriately manipulated here.
*/

class Mesh {
public:
	const char* name;
	std::vector <Vertex> vertices;
	std::vector <GLuint> indices;
	std::vector <Texture> textures;
	GLuint depthMap = 0;
	Mesh* gravSource; // body whose SOI this body is currently in / body which exerts the greatest grav field on body in solar system
	int soiID; // sphere of influence identification for 2 body equations
	int spiceID;
	int baryID;

	bool isLightSource;
	bool areRings;
	glm::vec4 Color; // only if object is a light source
	glm::vec3 Pos; // world position
	glm::vec3 oPos; // holder for trails
	glm::vec3 rPos; // relative posiition to gravitational source
	glm::vec3 sphPos;
	glm::vec3 Vel; // world velocity
	glm::mat4 Model;
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




	Mesh(const char* objName, std::vector<Vertex> vertices, std::vector <GLuint> indices, std::vector <Texture> textures, bool isLight, bool areRings, glm::vec4 objColor, glm::vec3 objPos, Shader *shaderProgram, int baryIDx, int spiceIDx);

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

	// calculate orbital position
	void Orbit(Mesh* lightSource, double UTCTime);  // MIGHT HAVE TO HOLD VELOCITY AND PARENT SOURCE AS MESH PROPERTY, GET LIGHT SOURCES FROM SYSTEM

	// update model position and orientation
	void updateModel(Mesh& source);

	
};

#endif
