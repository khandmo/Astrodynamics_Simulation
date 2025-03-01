#pragma once
#ifndef GUI_H
#define GUI_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <vector>
#include "System.h"

class Mesh; // "foward declaration" solved issues with the struct not recognizing
class Camera;
class ArtSat;
struct pvUnit;

struct GUIData {
	char* time;
	int tWRange;
	int* tW;
	double* simTime;

	std::vector <Mesh*> bodies;

	Camera* camera;

	std::vector<ArtSat> *artSatAll;
};


class GUI {
public:

	const char** bodyNames = nullptr;
	int bodyNamesLen;

	const char** satNames = nullptr;
	int satNamesLen;

	int focusType = 0; // locally designates difference between planet and sat focus
	int satFocusIndex = 0;

	ArtSat* sat;
	pvUnit* pv;

	bool newArtSat = false;
	bool newMan = false;
	// new artificial satellite parameters
	glm::vec3 initPos = { 415, glm::pi<float>() / 2, 0 };
	glm::vec3 initVel = { 7.67, glm::pi<float>() / 2, 3 * glm::pi<float>() / 2 };

	ArtSat* copySat = nullptr;

	
	bool showMan = false;
	const char** manList = nullptr;
	int manListLen = 0;
	int manListCurr = 0;

	glm::vec3 manData = { 0, 0, 0 };
	glm::vec3 oldManData = manData;
	
	float newManDt = 0;
	float oldManDt = 0;
	float manDt = 0;
	double goToManTime = -1;

	GUI(GLFWwindow* window, GUIData guiData);

	void guiLoopStart(GUIData guiData);

	void guiLoopADS(GUIData guiData);

	void guiLoopEnd(GUIData guiData);

	void guiDestroy();

};

#endif