#include "Camera.h"

glm::vec3 sphToCart(glm::vec3 sphCoords);

Camera::Camera(int width, int height, glm::vec3 position) {
	Camera::width = width;
	Camera::height = height;
	Position = position;
	OrigPos = position;

	// initialize focusDistSeg
	focusDistSeg[0] = 1.25;
	for (int i = 1; i < 50; i++) {
		if (i <= 25)
			focusDistSeg[i] = focusDistSeg[i - 1] + 0.25;
		else
			focusDistSeg[i] = focusDistSeg[i - 1] + 0.75;
	}
}

void Camera::updateMatrix(float FOVdeg, float nearPlane, float farPlane) {
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 proj = glm::mat4(1.0f);

	// lookAt calculates 4D matrix to normalize objects based on
	// the position of the camera, the direction the camera is looking,
	// and an up vector
	view = glm::lookAt(Position, Position + Orientation, Up);
	Camera::view = view;

	// calculate projection normally
	proj = glm::perspective(glm::radians(FOVdeg), (float)(width / height), nearPlane, farPlane);
	Camera::proj = proj;

	cameraMatrix = proj * view;
}

void Camera::Matrix(Shader& shader, const char* uniform) {
	// send camera matrix to model shaders
	glUniformMatrix4fv(glGetUniformLocation(shader.ID, uniform), 1, GL_FALSE, glm::value_ptr(cameraMatrix));
}

void Camera::smoothInputs(GLFWwindow* window, std::vector<glm::vec3*> &bodyPos) {
	//keyboard controls
	if (!focusMode) {
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) { // forward
			Position += speed * Orientation;
		}
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) { // backward
			Position -= speed * Orientation;
		}
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) { // left
			Position -= speed * glm::normalize(glm::cross(Orientation, Up));
		}
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) { // right
			Position += speed * glm::normalize(glm::cross(Orientation, Up));
		}
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) { // up
			Position += speed * Up;
		}
		if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) { // down
			Position -= speed * Up;
		}
		if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) { // faster
			speed = 0.1f;
		}
		if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE) { // normal speed
			speed = 0.01f;
		}
	}
	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) { // reset spatial and cam position
		if (cameraViewCycle == 0) { 
			Position = glm::vec3(1000.0f, 0.0f, 0.0f);
			Orientation = glm::vec3(-1.0f, 0.0f, 0.0f);
			Up = glm::vec3(0.0f, 1.0f, 0.0f);
		}
		else if (cameraViewCycle == 1) {
			Position = glm::vec3(0.0f, 3000.0f, 0.0f);
			Orientation = glm::vec3(0.0f, -1.0f, 0.0f);
			Up = glm::vec3(0.0f, 0.0f, -1.0f);
		}
	}
	// hold focus on body
	if (focusMode == true) {
		Position = *bodyPos[focusBody] + focusPos;
	}

	
	//mouse controls
	if (!focusMode) {
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
			// hide cursor on press
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

			// for smooth mouse look around
			if (firstClick) {
				glfwSetCursorPos(window, (width / 2), (height / 2));
				firstClick = false;
			}

			// get mouse pos
			double mouseX;
			double mouseY;
			glfwGetCursorPos(window, &mouseX, &mouseY);

			// spin factors
			float rotx = sensitivity * (float)(mouseY - (height / 2)) / height;
			float roty = sensitivity * (float)(mouseX - (height / 2)) / height;

			// prevent barrel rolls - calculate new orientation before it happens
			glm::vec3 newOrientation = glm::rotate(Orientation, glm::radians(-rotx), glm::normalize(glm::cross(Orientation, Up)));

			// if new orientation close to up or down axis, prevent orientation from reaching it
			if (!((glm::angle(newOrientation, Up) <= glm::radians(5.0f) or glm::angle(newOrientation, -Up) <= glm::radians(5.0f)))) {
				Orientation = newOrientation;
			}

			// calculate y rotation
			Orientation = glm::rotate(Orientation, glm::radians(-roty), Up);

			// keep cursor in middle of screen
			glfwSetCursorPos(window, (width / 2), (height / 2));

		}
		else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE) {
			// reveal cursor on release
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			firstClick = true;
		}
	}
	else { // focus mode enables mouse rotation around body in focus
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
			// hide cursor on press
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

			// for smooth mouse look around
			if (firstClick) {
				glfwSetCursorPos(window, (width / 2), (height / 2));
				firstClick = false;
			}

			// get mouse pos
			double mouseX;
			double mouseY;
			glfwGetCursorPos(window, &mouseX, &mouseY);

			// spin factors
			float rotx = sensitivity * (float)(mouseY - (height / 2)) / height;
			float roty = sensitivity * (float)(mouseX - (height / 2)) / height;

			// NOT UPDATING THE POSITION AT ALL, SHOULD UPDATE AROUND (0, 0, 0) FOR FOCUS POSITION, THEN UPDATE RELATIVE TO WORLD ORIGIN

			// rotate position around body with fixed r by rotation amounts and compensate orientation
			// prevent barrel rolls - calculate new orientation before it happens
			glm::vec3 newPosition = glm::rotate(focusPos, glm::radians(rotx), glm::normalize(glm::cross(focusPos, Up)));
			glm::vec3 newOrientation = glm::rotate(Orientation, glm::radians(rotx), glm::normalize(glm::cross(focusPos, Up)));

			// if new position close to up or down axis, prevent position from reaching it
			if (!((glm::angle(newOrientation, Up) <= glm::radians(5.0f) or glm::angle(newOrientation, -Up) <= glm::radians(5.0f)))) {
				focusPos = newPosition;
				Orientation = newOrientation;
			}

			// calculate y rotation and orientation
			focusPos = glm::rotate(focusPos, glm::radians(-roty), Up);
			Position = *bodyPos[focusBody] + focusPos;
			Orientation = glm::rotate(Orientation, glm::radians(-roty), Up);
			

			// keep cursor in middle of screen
			glfwSetCursorPos(window, (width / 2), (height / 2));

		}
		else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE) {
			// reveal cursor on release
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			firstClick = true;
		}
	}
}

void Camera::hardInputs(GLFWwindow* window, std::vector<glm::vec3*> &bodyPos, std::vector<float> &bodyRadii) {
	if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS) { // cycle camera positions
		// should change main body based on focus mode
		if (cameraViewCycle == 0) { // top down
			Position = glm::vec3(0.0f, 3000.0f, 0.0f);
			Orientation = glm::vec3(0.0f, -1.0f, 0.0f);
			Up = glm::vec3(0.0f, 0.0f, -1.0f);
			cameraViewCycle = 1;
		}
		else if (cameraViewCycle == 1) { // planar
			Position = glm::vec3(1000.0f, 0.0f, 0.0f);
			Orientation = glm::vec3(-1.0f, 0.0f, 0.0f);
			Up = glm::vec3(0.0f, 1.0f, 0.0f);
			cameraViewCycle = 0;
		}
	}

	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {  // toggle focus mode on camera
		focusMode = !focusMode;
		if (focusMode == true) {
			// reset focus parameters
			focusBody = 0;
			focusPos = glm::vec3((float)(focusDistSeg[focusDistMarker] * bodyRadii[focusBody]), 0.0f, 0.0f); // initialized as default focusDistMarker
			// reset orientation to look at the center of object, regardless of position
			Orientation = glm::vec3(-1.0f, 0.0f, 0.0f);
		}
	}
	if (focusMode) {
		// scroll through object in focus
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) { // move backward through object list focus and preserve camera distance to body
			if (focusBody == 0) {
				focusBody = bodyPos.size() - 1;
				focusPos *= (bodyRadii[focusBody] / bodyRadii[0]);
			}
			else {
				focusBody--;
				focusPos *= (bodyRadii[focusBody] / bodyRadii[focusBody + 1]);
			}
		}
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) { // move forward through object list focus and preserve camera distance to body
			if (focusBody == bodyPos.size() - 1) {
				focusBody = 0;
				focusPos *= (bodyRadii[focusBody] / bodyRadii[bodyPos.size() - 1]);
			}
			else {
				focusBody++;
				focusPos *= (bodyRadii[focusBody] / bodyRadii[focusBody + 1]);
			}
		}		
		// focus mode zoom in / zoom out
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) { // zoom in
			if (focusDistMarker > 0)
				focusPos *= (focusDistSeg[focusDistMarker - 1] / focusDistSeg[focusDistMarker--]);
		}
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) { // zoom out
			if (focusDistMarker < 50)
				focusPos *= (focusDistSeg[focusDistMarker + 1] / focusDistSeg[focusDistMarker++]);
		}
	}
}


glm::vec3 sphToCart(glm::vec3 sphCoords) {
	// turns (r, theta, phi) to (x, y, z)
	glm::vec3 cartCoords = glm::vec3(0, 0, 0);
	cartCoords.z = sphCoords.x * sin(sphCoords.y) * cos(sphCoords.z);
	cartCoords.x = sphCoords.x * sin(sphCoords.y) * sin(sphCoords.z);
	cartCoords.y = sphCoords.x * cos(sphCoords.y);
	return cartCoords;
}