#pragma once
#ifndef GUI_H
#define GUI_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <vector>
#include <future>
#include <iostream>
#include "System.h"


class Mesh; // "foward declaration" solved issues with the struct not recognizing
class Camera;
class System;
class ArtSat;
struct pvUnit;


struct GUIData {
	char* time;
	int tWRange;
	int* tW;
	double* simTime;

	System* Sys;
};


class GUI {
public:

	const char** bodyNames = nullptr;
	int bodyNamesLen;

	const char** satNames = nullptr;
	int satNamesLen;

	int focusType = 0; // locally designates difference between planet and sat focus
	int satFocusIndex = 0;

	ArtSat* sat = nullptr;
	pvUnit* pv = nullptr;

	bool newArtSat = false;
	bool newMan = false;
	// new artificial satellite parameters
	glm::vec3 initPos = { 415, glm::pi<float>() / 2, 0 };
	glm::vec3 initVel = { 7.67, glm::pi<float>() / 2, glm::pi<float>() / 2 };

	ArtSat* copySat = nullptr;

	
	bool showMan = false;
	bool targetWin = false;
	bool retrograde = false;
	bool fineCtrl = false;
	
	const char** manList = nullptr;
	int manListLen = 0;
	int manListCurr = 0;

	glm::vec3 manData = { 0, 0, 0 };
	glm::vec3 oldManData = manData;
	glm::vec3 fineCtrlData = { 0, 0, 0 };

	std::atomic<bool> manStopBool = false;
	
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