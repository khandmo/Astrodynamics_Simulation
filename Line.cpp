#include "Line.h"


Line::Line() { // empty default constructor

}

Line::Line(std::vector<glm::vec3> path){ // save GLuint pathVBO and num of path points

	GLuint pathVBOtemp;
    VAO pathVAOtemp;
	glGenBuffers(1, &pathVBOtemp);
    glBindVertexArray(pathVAOtemp.ID);

	glBindBuffer(GL_ARRAY_BUFFER, pathVBOtemp);
	glBufferData(GL_ARRAY_BUFFER, path.size() * sizeof(glm::vec3), &path[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    Line::pathVAO = pathVAOtemp.ID;
    Line::pathVBO = pathVBOtemp;

}


void Line::renderLine(Shader shaderProgram, size_t numPathPoints, glm::mat4 model, glm::mat4 view, glm::mat4 projection) { // use shader, pathVBO, and input to render path in renderer

	shaderProgram.Activate();
    glBindVertexArray(pathVAO);

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
    glBindVertexArray(0);
    glUseProgram(0);
}