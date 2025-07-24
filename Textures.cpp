#include"Textures.h"

Texture::Texture(const char* picturePath, const char* texType, GLuint slot, GLenum format, GLenum pixelType) {
	//develop TEXTURE ID
	type = texType;
	//generate texture
	glGenTextures(1, &ID); 

	//bind texture / assign to texture unit
	glActiveTexture(GL_TEXTURE0 + slot); // GL_TEXTURE0 + 0
	unit = slot;
	glBindTexture(GL_TEXTURE_2D, ID);

	// set texture repeating
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	
	// set texture algorithm for shrink/enlarge
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	//load and generate texture from png
	int width, height, numChannels;
	//flips image so it is right-side up
	stbi_set_flip_vertically_on_load(true);
	//read image from file and store it
	unsigned char* data = stbi_load(picturePath, &width, &height, &numChannels, 0);

	
	// assign image to texture object
	if (data) {
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, format, pixelType, data);
		glGenerateMipmap(GL_TEXTURE_2D);
	}
	else {
		std::cout << "Failed to load texture" << std::endl;
	}
	
	// free data
	stbi_image_free(data);
	//unbind texture object to prevent modification
	glBindTexture(GL_TEXTURE_2D, 0); 
}

void Texture::texUnit(Shader& shader, const char* uniform, GLuint unit) {
	// gets location of uniform
	GLuint texUni = glGetUniformLocation(shader.ID, uniform);
	// activate shader
	shader.Activate();
	// set value of uniform
	glUniform1i(texUni, unit);
}

void Texture::Bind() {
	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_2D, ID);
}

void Texture::Unbind() {
	glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::Delete() {
	glDeleteTextures(1, &ID);
}


