#ifndef VAO_CLASS_H
#define VAO_CLASS_H

#include <glad/glad.h>
#include "VBO.h"

/*
The VAO class sets up vertex arrtibute objects and can link attributes, bind and unbind, and delete itself when called. 
Holds ID to be manipulated from the outside.
*/

class VAO {
public:
	// ID reference to VAO
	GLuint ID;
	// Constructor
	VAO();

	// Links VBO to VAO using a given layout
	void LinkAttrib(VBO& VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset);
	void Bind();
	void Unbind();
	void Delete();
};


#endif
