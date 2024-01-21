#include "VAO.h"

VAO::VAO() {
	glGenVertexArrays(1, &ID);
}

void VAO::LinkAttrib(VBO& VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset) {
	/*
	VAO stores vertex attribute configs
	and which VBO's to use for that attribute
	-
	Whenever an object needs to be drawn, instead of 
	redoing the VBO code, the VAO just binds to that
	already processed VBO data to be used immediately
	*/
	VBO.Bind();
	/*
	Tells OpenGL how to interpret vertex data before rendering
	Parameters:
	which vertex attribute to configure
	number of vertex attributes (1 3-vertex)
	type of vector data stored
	if we want the data to be normalized (false, already)
	stride - space in memory between consecutive vertex attribs
	offset of where pos data begins in buffer
	*/
	glVertexAttribPointer(layout, numComponents, type, GL_FALSE, stride, offset);
	glEnableVertexAttribArray(layout);
	VBO.Unbind();
}

void VAO::Bind() {
	glBindVertexArray(ID);
}

void VAO::Unbind() {
	glBindVertexArray(0);
}

void VAO::Delete() {
	glDeleteVertexArrays(1, &ID);
}