#ifndef LINE_H__
#define LINE_H__

#include "VBO.h"
#include "VAO.h"
#include "Shaders.h"
#include <glm/gtc/type_ptr.hpp>
#include <vector>


class Line {
public:
	GLuint pathVBO;
	GLuint pathVAO;

	Line();
	Line(std::vector<glm::vec3> path);

	void renderLine(Shader shaderProgram, size_t numPathPoints, glm::mat4 model, glm::mat4 view, glm::mat4 projection); // use shader, pathVBO, and input to render path in renderer

};

#endif
