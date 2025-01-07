#pragma once
#ifndef GUI_H
#define GUI_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <vector>
#include "Mesh.h"

struct Mesh; // "foward declaration" solved issues with the struct not recognizing

struct GUIData {
	char* time;
	int tWRange;
	int* tW;

	std::vector <Mesh*> bodies;

	float* camSpeed;
	bool* focusMode;
	int* focusBody;
};


class GUI {
public:

	const char** bodyNames = nullptr;
	int bodyNamesLen;

	GUI(GLFWwindow* window, GUIData guiData);

	void guiLoopStart(GUIData guiData);

	void guiLoopADS(GUIData guiData);

	void guiLoopEnd();

	void guiDestroy();

};

#endif