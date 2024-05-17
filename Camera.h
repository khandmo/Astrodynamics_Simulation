#ifndef CAMERA_CLASS_H
#define CAMERA_CLASS_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <vector>


#include "Shaders.h"

/*
The camera class handles the manipulation of the camera matrices for position and orientation. It also handles
user inputs for movement of the camera and focus mode / view.
*/

class Camera {
public:
	int cameraViewCycle = 0;
	bool focusMode = false;
	int focusBody = 0;
	glm::vec3 focusPos = glm::vec3(3.0f, 0.0f, 0.0f);
	float focusDistSeg[50];
	int focusDistMarker = 20;

	glm::vec3 Position;
	glm::vec3 OrigPos;
	glm::vec3 Orientation = glm::vec3(-1.0f, 0.0f, 0.0f);
	glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 proj = glm::mat4(1.0f);
	glm::mat4 cameraMatrix = glm::mat4(1.0f);

	// for smooth mouse moves
	bool firstClick = true;
	bool firstPress = true;
	std::vector<bool> keyRange;

	int width;
	int height;

	float speed = 0.01f;
	float sensitivity = 100.0f;

	Camera(int width, int height, glm::vec3 position);

	void updateMatrix(float FOVdeg, float nearPlane, float farPlane);

	void Matrix(Shader& shader, const char* uniform);
	
	void smoothInputs(GLFWwindow* window, std::vector<glm::vec3*> &bodyPos);

	void hardInputs(GLFWwindow* window, std::vector<glm::vec3*>& bodyPos, std::vector<float>& bodyRadii);

	// handles press and release for robust button press recognition
	bool keyPress(GLFWwindow* window, int key);
};


#endif
