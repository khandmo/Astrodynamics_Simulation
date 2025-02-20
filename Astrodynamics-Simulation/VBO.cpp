#include "VBO.h"
// all were snatched from main to be organized for building
// VBO's dynamically. Each keyword "VBO" changed to "ID"

VBO::VBO(std::vector <Vertex>& vertices) {
	/*
	VBO stores vertices in GPU memory
	glGenBuffers generates the buffer to the reference ID
	BindBuffer binds specific reference to keyword "GL_ARRAY_BUFFER"
	BufferData assigns vertex data to this buffer with
	parameters:
	Buffer itself
	size of data
	data
	how the gpu should manage data
	*/
	glGenBuffers(1, &ID);
	glBindBuffer(GL_ARRAY_BUFFER, ID);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
}

void VBO::Bind() {
	glBindBuffer(GL_ARRAY_BUFFER, ID);
}

void VBO::Unbind() {
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VBO::Delete() {
	glDeleteBuffers(1, &ID);
}