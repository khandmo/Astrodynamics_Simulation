#include "Line.h"


GLuint setLine(std::vector<glm::vec3> path){ // save GLuint pathVBO and num of path points

	GLuint pathVBO;
	glGenBuffers(1, &pathVBO);
	glBindBuffer(GL_ARRAY_BUFFER, pathVBO);
	glBufferData(GL_ARRAY_BUFFER, path.size() * sizeof(glm::vec3), &path[0], GL_STATIC_DRAW);
    
    return pathVBO;
}


void renderLine(Shader shaderProgram, GLuint pathVBO, size_t numPathPoints, glm::mat4 model, glm::mat4 view, glm::mat4 projection) { // use shader, pathVBO, and input to render path in renderer

	shaderProgram.Activate();

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram.ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram.ID, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram.ID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

	glBindBuffer(GL_ARRAY_BUFFER, pathVBO);

    GLint posAttrib = glGetAttribLocation(shaderProgram.ID, "position");
    glEnableVertexAttribArray(posAttrib);
    glVertexAttribPointer(posAttrib, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);

    glDrawArrays(GL_LINE_STRIP, 0, numPathPoints);

    glDisableVertexAttribArray(posAttrib);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}