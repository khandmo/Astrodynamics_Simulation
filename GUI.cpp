#include "GUI.h"

char* secToDay(double dt);
void arrowToggle(float& value, bool isInt);

GUI::GUI(GLFWwindow* window, GUIData guiData) {

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	bodyNames = new const char*[guiData.Sys->bodies.size()]; // delete this in deconstructor
	int counter = 0;
	for (auto& body : guiData.Sys->bodies) {
		if (!body->areRings) {
			bodyNames[counter] = body->name;
			counter++;
		}
	}
	bodyNamesLen = counter;

	counter = 0;
	satNames = new const char* [guiData.Sys->artSats.size()];
	for (auto& sat : guiData.Sys->artSats) {
		satNames[counter] = sat.name;
		counter++;
	}
	satNamesLen = counter;
	
}

void GUI::guiLoopStart(GUIData guiData) {
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	//ImGui::ShowDemoWindow();
	guiLoopADS(guiData);
}

void GUI::guiLoopADS(GUIData guiData) {
	// AstroDynSim GUI detail
	bool* p_open = 0;
	static bool no_titlebar = false;
	static bool no_scrollbar = false;
	static bool no_menu = false;
	static bool no_move = false;
	static bool no_resize = false;
	static bool no_collapse = false;
	static bool no_close = false;
	static bool no_nav = false;
	static bool no_background = false;
	static bool no_bring_to_front = false;
	static bool unsaved_document = false;

	ImGuiWindowFlags window_flags = 0;
	if (no_titlebar)        window_flags |= ImGuiWindowFlags_NoTitleBar;
	if (no_scrollbar)       window_flags |= ImGuiWindowFlags_NoScrollbar;
	if (!no_menu)           window_flags |= ImGuiWindowFlags_MenuBar;
	if (no_move)            window_flags |= ImGuiWindowFlags_NoMove;
	if (no_resize)          window_flags |= ImGuiWindowFlags_NoResize;
	if (no_collapse)        window_flags |= ImGuiWindowFlags_NoCollapse;
	if (no_nav)             window_flags |= ImGuiWindowFlags_NoNav;
	if (no_background)      window_flags |= ImGuiWindowFlags_NoBackground;
	if (no_bring_to_front)  window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;
	if (unsaved_document)   window_flags |= ImGuiWindowFlags_UnsavedDocument;
	if (no_close)           p_open = NULL; // Don't pass our bool* to Begin

	const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(ImVec2(main_viewport->WorkPos.x + 650, main_viewport->WorkPos.y + 20), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

	if (!ImGui::Begin("Astrodynamic Simulator", p_open, window_flags))
	{
		// Early out if the window is collapsed, as an optimization.
		ImGui::End();
		return;
	}
	ImGui::SeparatorText("TIME CONTROL");
	float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
	ImGui::Text("Time Warp");
	ImGui::SameLine();
	ImGui::PushItemFlag(ImGuiItemFlags_ButtonRepeat, true);
	if (ImGui::ArrowButton("##left", ImGuiDir_Left)) { (*(guiData.tW))--; }
	ImGui::SameLine();
	ImGui::Text("%d%s", guiData.tWRange, "x");
	ImGui::SameLine();
	if (ImGui::ArrowButton("##right", ImGuiDir_Right)) { (*(guiData.tW))++; } // when tw value gets bigger, this moves to compensate, should fix pos
	ImGui::PopItemFlag();
	ImGui::SameLine();
	ImGui::Text("Date : %s", guiData.time);

	if (ImGui::Button("Kill TimeWarp"))
		*(guiData.tW) = 15;




	ImGui::SeparatorText("FOCUS CONTROL");
	if (ImGui::Button("Planet Focus")) {
		if (guiData.Sys->camera->focusMode == false)
			guiData.Sys->camera->focusMode = true;
		else if (focusType == 1)
			guiData.Sys->camera->focusMode = false;
		guiData.Sys->camera->focusBody = 0;
		focusType = 1;
	}

	ImGui::SameLine();
	if (ImGui::Button("Sat Focus")) {
		if (guiData.Sys->artSats.size() > 0) {
			if (guiData.Sys->camera->focusMode == false)
				guiData.Sys->camera->focusMode = true;
			else if (focusType == 2)
				guiData.Sys->camera->focusMode = false;
			guiData.Sys->camera->focusBody = -1;
			focusType = 2;
		}
	}


	if (guiData.Sys->camera->focusMode && focusType == 1) {
		if (guiData.Sys->camera->focusBody < 0)
			guiData.Sys->camera->focusBody = 0;
		ImGui::Combo("Body Select", &(guiData.Sys->camera->focusBody), bodyNames, bodyNamesLen);
	}

	if (guiData.Sys->camera->focusMode && focusType == 2) {
		if (guiData.Sys->camera->focusBody > -1)
			guiData.Sys->camera->focusBody = -1;

		 // ************ isnt going to work, need way to access index without the math
		ImGui::Combo("Sat Select", &satFocusIndex, satNames, satNamesLen);
		guiData.Sys->camera->focusBody = -1 - satFocusIndex;
	}

	if (guiData.Sys->camera->focusMode == false)
		focusType = 0;







	ImGui::SeparatorText("ART SAT CONTROL");

	if (focusType == 2) {
		ArtSat* currSat = &(guiData.Sys->artSats)[-guiData.Sys->camera->focusBody - 1];
		
		/*
		// if sat out of scope
		while (!currSat->inTime) {
			guiData.Sys->camera->focusBody--;
			currSat = &(guiData.Sys->artSats)[-guiData.Sys->camera->focusBody - 1];
		}
		*/

		// enable maneuver list in GUI
		if (manList == nullptr || guiData.Sys->camera->focusBody != guiData.Sys->camera->lastFocusBody ||
			manListLen != currSat->maneuvers.size()) {

			if (manList != nullptr) delete manList;
			manListLen = currSat->maneuvers.size();
			manList = new const char* [manListLen];
			for (int i = 0; i < currSat->maneuvers.size(); i++) {
				manList[i] = currSat->maneuvers[i].name;
			}
		}

		// focus sat info - apo/peri, time to each, MET, orbital period, maneuver info, maneuver button, refresh button
		if (currSat->stat != nullptr) {
			ImGui::Text("Altitude: %.2f km", currSat->stat->distToSoi);
			ImGui::Text("Apoapsis: %.2f km", currSat->stat->apoapsis);
			ImGui::Text("\tT%s", secToDay(currSat->stat->timeToApo));
			ImGui::Text("Periapsis: %.2f km", currSat->stat->periapsis);
			ImGui::Text("\tT%s", secToDay(currSat->stat->timeToPeri));
			ImGui::Text("MET %s", secToDay(currSat->stat->MET));
			ImGui::Text("Orbital Period %s", secToDay(currSat->stat->orbitalPeriod));
		}
		else {
			ImGui::Text("No data available");
		}

		if (ImGui::Button("Show Maneuvers")) {
			showMan = !showMan;
			manListCurr = 0;
		}

		// maneuver stats
		if (showMan) {
			ImGui::Combo("->", &manListCurr, manList, manListLen);
			ImGui::SameLine();
			if (ImGui::Button("Go-To")) { 
				goToManTime = currSat->maneuvers[manListCurr].time;
			}

			ImGui::Text("%s", currSat->maneuvers[manListCurr].desc);
			ImGui::Text("deltaV: %.2f m/s", glm::length(currSat->maneuvers[manListCurr].newState.Vel - currSat->maneuvers[manListCurr].origState.Vel) * 1000);
			ImGui::Text("Burn in T%s", secToDay(currSat->maneuvers[manListCurr].time - *guiData.simTime));
			
		}

		if (ImGui::Button("Refresh Orbit")) {
			currSat->refreshTraj(guiData.Sys->bodies, *guiData.simTime);
		}
		ImGui::SameLine();
		if (ImGui::Button("New Maneuver")) {
			if (newMan == true) { // if already open, reset & close
				delete copySat;
				copySat = nullptr;
				newMan = false;
				manDt = 0;
				manData = { 0, 0, 0 };
			}
			else {
				newMan = true;
				newManDt = *guiData.simTime + (60 * 5);
				copySat = new ArtSat();
				*copySat^* currSat;
			}

		}
		if (newMan){ // maneuver logic

			static char str0[30] = "maneuver 1"; // modify to tick up with amt of maneuvers
			ImGui::InputText("Name", str0, IM_ARRAYSIZE(str0));

			static char str1[30] = "";
			ImGui::InputText("Description", str1, IM_ARRAYSIZE(str1));

			ImGui::SliderFloat("Time", &manDt, 0, currSat->stat->orbitalPeriod, "%.f s");
			ImGui::SameLine();
			arrowToggle(manDt, true);
			if (manDt < 0) manDt += currSat->stat->orbitalPeriod;

			manData[0] *= 1000;
			ImGui::SliderFloat("Velocity", &manData[0], 0, 8000, "%.2f m/s"); // ******** need way to burn retrograde
			ImGui::SameLine();
			arrowToggle(manData[0], true);
			if (manData[0] < 0) manData[0] += 12;
			manData[0] /= 1000;

			ImGui::SliderFloat("Pitch", &manData[1], 0, glm::pi<float>(), "%.2f rad");
			ImGui::SameLine();
			arrowToggle(manData[1], false);
			if (manData[1] < 0) manData[1] += glm::pi<float>();

			ImGui::SliderFloat("Yaw", &manData[2], 0, 2 * glm::pi<float>(), "%.2f rad");
			ImGui::SameLine();
			arrowToggle(manData[2], false);
			if (manData[2] < 0) manData[2] += 2 * glm::pi<float>();

			// if maneuver changed do new calculation
			if (oldManDt != manDt || oldManData != manData) {
				// handle thread atomic bool
				manStopBool.store(true);
				manStopBool.store(false);

				while (guiData.Sys->maneuverThread.valid()) { // make sure thread is dead
					if (guiData.Sys->maneuverThread.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
						guiData.Sys->maneuverThread.get();
					}
				}

				delete copySat;
				copySat = new ArtSat();
				*copySat^* currSat;

				// solve new maneuver
				copySat->ArtSatManeuver(manData, guiData.Sys->bodies, manStopBool, newManDt + manDt, str0, str1);
			}
			if (guiData.Sys->maneuverThread.valid()) {
				// fill up temp buffer for early render
				if (guiData.Sys->maneuverThread.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
					copySat->fillBuff(copySat->dynBuff, copySat->dynTimes);
				}
				else {
					// or if done processing empty thread
					guiData.Sys->maneuverThread.get();
					copySat->threadStop = nullptr;
				}
			}
			// render and update ( add modifier to change orbit color, fade, + update and render actual sat at same time )
			copySat->ArtSatUpdState(guiData.Sys->bodies, *guiData.simTime, *guiData.tW, 1); // right time????

			oldManDt = manDt;
			oldManData = manData;
			// save maneuver
			if (ImGui::Button("Save Maneuver")) {
				currSat->maneuvers.push_back({ (copySat->maneuvers[copySat->maneuvers.size() - 1].origState), 
					(copySat->maneuvers[copySat->maneuvers.size() - 1].newState), newManDt + manDt, str0, str1});
				delete copySat;
				copySat = nullptr;
				manDt = 0;
				manData = { 0, 0, 0 };
				newMan = false;
			}
		}
	}






	
	if (ImGui::Button("New Artificial Satellite")) {
		newArtSat = true;
		sat = new ArtSat();
		pv = new pvUnit;
		sat->sysThread = &guiData.Sys->maneuverThread;
	}
	if (newArtSat) { // new satellite logic

		static char str0[30] = "Untitled Spacecraft 1"; // word input, also use to display info
		ImGui::InputText("Name", str0, IM_ARRAYSIZE(str0));
		
		int dummy = (int)initPos[0];
		ImGui::SliderInt("r", &dummy, 300, 10000, "%d km"); // when r changes, vMag should change to keep the same orbital energy over some threshold
		initPos[0] = dummy;
		ImGui::SameLine();
		arrowToggle(initPos[0], true);
		
		ImGui::SliderFloat("theta", &initPos[1], 0, glm::pi<float>(), "%.2f rad");
		ImGui::SameLine();
		arrowToggle(initPos[1], false);
		if (initPos[1] < 0) initPos[1] += glm::pi<float>();

		ImGui::SliderFloat("phi", &initPos[2], 0, 2 * glm::pi<float>(), "%.2f rad");
		ImGui::SameLine();
		arrowToggle(initPos[2], false);
		if (initPos[2] < 0) initPos[2] += 2 * glm::pi<float>();

		ImGui::SliderFloat("Velocity", &initVel[0], 0, 12, "%.2f m/s");
		ImGui::SameLine();
		arrowToggle(initVel[0], false);
		if (initVel[0] < 0) initVel[0] += 12;
		
		ImGui::SliderFloat("Pitch", &initVel[1], 0, glm::pi<float>(), "%.2f rad");
		ImGui::SameLine();
		arrowToggle(initVel[1], false);
		if (initVel[1] < 0) initVel[1] += glm::pi<float>();

		ImGui::SliderFloat("Yaw", &initVel[2], 0, 2 * glm::pi<float>(), "%.2f rad");
		ImGui::SameLine();
		arrowToggle(initVel[2], false);
		if (initVel[2] < 0) initVel[2] += 2 * glm::pi<float>();

		// init pv
		glm::vec3 cartDummy = sphToCart(initPos + glm::vec3{guiData.Sys->bodies[3]->realRadius, 0, 0});
		pv->Pos = glm::dvec3{ cartDummy.z, cartDummy.x, cartDummy.y};

		glm::vec3 relToPos = { 0, initPos[1] - glm::pi<float>() / 2, initPos[2] };
		cartDummy = sphToCart(initVel + relToPos);
		pv->Vel = glm::dvec3{ cartDummy.z, cartDummy.x, cartDummy.y };

		// process pv
		sat->ArtSatPlan(*pv, *(guiData.simTime), 3, guiData.Sys->bodies);
		sat->ArtSatUpdState(guiData.Sys->bodies, *guiData.simTime, *guiData.tW, -1);

		// reset art sat parameters to default values in each, delete pvUnit as well
		if (ImGui::Button("Save")) {
			bool repeatName = false;
			for (auto& someSat : guiData.Sys->artSats) {
				if (someSat.name == str0) repeatName = true;
			}
			if (!repeatName) {
				sat->name = str0;
				guiData.Sys->artSats.push_back(*sat);
				sat = nullptr;
				pv = nullptr;
				newArtSat = false;

				// change sat focus list 
				delete satNames;
				int counter = 0;
				satNames = new const char* [guiData.Sys->artSats.size()];
				for (auto& someSat : guiData.Sys->artSats) {
					satNames[counter] = someSat.name;
					counter++;
				}
				satNamesLen = counter;

				// reset init values
				initPos = { 300, glm::pi<float>() / 2, 0 };
				initVel = { 7.8, glm::pi<float>() / 2, 3 * glm::pi<float>() / 2 };
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Discard")) {
			sat->~ArtSat();
			newArtSat = false;
			sat = nullptr;
			pv = nullptr;
		}
		if (ImGui::Button("Print Orbit") && pv != nullptr) {
			std::cout << pv->Pos.x << " " << pv->Pos.y << " " << pv->Pos.z << '\n' << pv->Vel.x << " " << pv->Vel.y << " " << pv->Vel.z << '\n';
		}
	}

	
	ImGui::End();
}

void GUI::guiLoopEnd(GUIData guiData) {
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	if (newArtSat) {
		sat->ArtSatRender(guiData.Sys->camera, *(guiData.Sys->bodies[0]));
	}
	if (newMan) {
		copySat->ArtSatRender(guiData.Sys->camera, *(guiData.Sys->bodies[0]));
	}
}

void GUI::guiDestroy() {
	delete bodyNames;
	delete satNames;

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

void arrowToggle(float& value, bool isInt) {
	float inc = 0;
	if (isInt)
		inc = 1;
	else
		inc = 0.01;
	ImGui::PushItemFlag(ImGuiItemFlags_ButtonRepeat, true);
	if (ImGui::ArrowButton(std::to_string(rand()).c_str(), ImGuiDir_Left)) { value -= inc; }
	ImGui::SameLine();
	if (ImGui::ArrowButton(std::to_string(rand()).c_str(), ImGuiDir_Right)) { value += inc; } // when tw value gets bigger, this moves to compensate, should fix pos
	ImGui::PopItemFlag();
}

// configured for T -/+ display
char* secToDay(double dt) {
	bool neg = false;
	if (dt < 0) {
		dt = -dt;
		neg = true;
	}

	int min = 60;
	int hour = 60;
	int day = 24;

	int numMin = 0;
	int numHour = 0;
	int numDay = 0;

	while (dt > day * hour * min) {
		dt -= (day * hour * min);
		numDay++;
	}
	while (dt > hour * min) {
		dt -= (hour * min);
		numHour++;
	}
	while (dt > min) {
		dt -= min;
		numMin++;
	}
	static char res[30];

	if (neg)
		sprintf(res, "+ %dd, %dh, %dm, %ds", numDay, numHour, numMin, (int)dt);
	else
		sprintf(res, "- %dd, %dh, %dm, %ds", numDay, numHour, numMin, (int)dt);
	
	return res;
}