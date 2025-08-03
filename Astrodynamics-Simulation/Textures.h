#ifndef TEXTURE_CLASS_H
#define TEXTURE_CLASS_H

#include <glad/glad.h>
#include <string>
#include <stb/stb_image.h>
#include <cerrno>
#include "Shaders.h"

/*
The texture class holds textures that can be used by the mesh class for wrapping. 
Function texUnit sends the texture information to the appropriate fragment shader uniform.
Has functions for binding and unbinding and deletion.
Holds ID for outside manipulation.
*/

class Texture {
public:
	GLuint ID;
	const char* type;
	GLuint unit;

	Texture(const char* picturePath, const char* texType, GLuint slot, GLenum format, GLenum pixelType);

	void texUnit(Shader& shader, const char* uniform, GLuint unit);

	void Bind();

	void Unbind();

	void Delete();
};

#endif

