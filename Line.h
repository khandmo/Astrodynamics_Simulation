#ifndef LINE_H__
#define LINE_H__

#include "VBO.h"
#include "Shaders.h"
#include <glm/gtc/type_ptr.hpp>
#include <vector>



GLuint setLine(std::vector<glm::vec3> path); // save GLuint pathVBO and num of path points

void renderLine(Shader shaderProgram, GLuint pathVBO, size_t numPathPoints, glm::mat4 model, glm::mat4 view, glm::mat4 projection); // use shader, pathVBO, and input to render path in renderer


#endif
