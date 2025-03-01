#ifndef MARKER_H__
#define MARKER_H__

#include "Camera.h"
#include "Shaders.h"
#include "VAO.h"
#include <string>

class Object;
class Camera;

class Marker {
public:


	GLuint shaderProgram;

	glm::vec3 fixedPos;
	glm::vec3 simPos;
	glm::vec2 screenPos;

	GLuint VAO, VBO;

	int marker_type;
	glm::vec4 color = { 1.0f, 1.0f, 1.0f, 0.74f };

	Marker(int type, float size, glm::vec3 fixedPos, glm::vec3 corr);

	void MarkerRender(Camera* camera);

	~Marker();

};



#endif MARKER_H__