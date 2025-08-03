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
};


class GUI {
public:

	const char** bodyNames = nullptr;
	int bodyNamesLen;

	ArtSat* sat;
	pvUnit* pv;

	bool newArtSat = false;
	// new artificial satellite parameters
	int r = 300; float vMag = 8.5; // km, km/s
	float theta = glm::pi<float>() / 2; // from pole (equatorial orbit)
	float phi = 0;
	float vTheta = glm::pi<float>() / 2;
	float vPhi = glm::pi<float>() / 2;

	GUI(GLFWwindow* window, GUIData guiData);

	void guiLoopStart(GUIData guiData);

	void guiLoopADS(GUIData guiData);

	void guiLoopEnd(GUIData guiData);

	void guiDestroy();

};

#endif