#include "GUI.h"

char* secToDay(double dt);
void arrowToggle(float& value, bool isInt, bool fineCtrl);

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
		bodyNames[counter] = body->name;
		counter++;
	}
	bodyNamesLen = counter;

	counter = 0;
	satNames = new const char* [guiData.Sys->artSats.size()];
	for (auto& sat : guiData.Sys->artSats) {
		satNames[counter] = sat.name;
		counter++;
	}
	satNamesLen = counter;
	
	manStopBool = &(guiData.Sys->satManStop);
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
	ImGui::SameLine();
	if (ImGui::Button("Warp To System Time")) {
		guiData.Sys->ArgClockSet(guiData.Sys->sysTime.time_in_sec);
	}
	ImGui::SameLine();
	if (ImGui::Checkbox("Show Orbits", &orbitShow)) {
		
	}



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



	// have a reset sat values button in case of random focus mode switches 
	// worried about targ focus with planet focus change





	ImGui::SeparatorText("ART SAT CONTROL");

	if (focusType == 2) {
		ArtSat* currSat = &(guiData.Sys->artSats)[-guiData.Sys->camera->focusBody - 1];

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
			ImGui::SameLine();
			ImGui::Text("Velocity: %.2f km/s", glm::length(currSat->state->Vel));
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

		if (ImGui::Button("Target")) {
			targetWin = !targetWin;
		}

		ImGui::SameLine();

		if (ImGui::Button("Refresh Orbit") && currSat->inTime) {
			currSat->refreshTraj(guiData.Sys->bodies, *guiData.simTime);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextUnformatted("Runs trajectory chart");
			ImGui::EndTooltip();
		}

		if (targetWin) {
			ImGui::Begin("Target", &targetWin);

			ImGui::SeparatorText("TARGET ACQUISITION");

			// dummy sat for maneuver info
			ArtSat* someSat = currSat;
			if (newMan)
				someSat = copySat;

			if (ImGui::Button("Toggle")) {
				if (someSat->targStat == nullptr) {
					someSat->targStat = new targStats;
				}
				else {
					delete someSat->targStat;
					someSat->targStat = nullptr;
					targetFocus = false;
					transferWin = false;
					transfer = -1;
					transferTime = -1;
					synPeriod = -1;
				}
			}

			// handle target focus
			if (targetFocus && guiData.Sys->camera->focusBody < 0 && someSat->targStat != nullptr) {
				lastFocus = guiData.Sys->camera->focusBody;
				guiData.Sys->camera->focusBody = someSat->targStat->targetIdx;
			}
			else if (guiData.Sys->camera->focusBody > 0) {
				guiData.Sys->camera->focusBody = lastFocus;
				lastFocus = -1;
			}

			if (someSat->targStat != nullptr) {
				/*if (someSat->lastTargIdx != someSat->targStat->targetIdx) { // targ update check
					someSat->lastTargIdx = someSat->targStat->targetIdx;
				}*/
				ImGui::Combo("Target Select", &someSat->targStat->targetIdx, bodyNames, bodyNamesLen);

				if (ImGui::Button("Target Focus")) {
					targetFocus = !targetFocus;
				}

				someSat->targStat->soiRad = guiData.Sys->bodies[someSat->targStat->targetIdx]->soiRadius;
				if (someSat->targStat->closeAppr - someSat->targStat->soiRad < 0 && someSat->targStat->targetIdx != 0) {
					someSat->targStat->soiCapture = true;
					int nodeAcc = 0;
					for (int i = 0; i < someSat->lineBuffSect.size(); i++) {
						if (someSat->lineBuffSect[i].first == someSat->targStat->targetIdx) {
							if (i != 0) i--;
							someSat->targStat->timeToCapture = someSat ->lBTime[(((LINE_BUFF_SIZE_AS * (i)) + someSat->lineBuffSect[i].second) / 2) - 2] - *(guiData.simTime);
							break;
						}
					}
				}

				ImGui::Text("Closest Approach: %.2f km", someSat->targStat->closeAppr);
				ImGui::Text("\tT%s", secToDay(someSat->targStat->timeToCloseAppr));

				if (someSat->targStat->soiCapture)
					ImGui::Text("Capture in T%s", secToDay(someSat->targStat->timeToCapture)); // does not compute
				else
					ImGui::Text("Capture Miss: %.2f km", someSat->targStat->closeAppr - someSat->targStat->soiRad);

				// time to optimal transfer?
				// warp button to optimal transfer?
				if (ImGui::Button("Hohmann Transfer Windows")) {
					transferWin = !transferWin;
					if (!transferWin && transfer != -1)
						transfer = -1;
				}

				if (transferWin) {
					// give closest future automatically
					ImGui::Text("Transfer %d\t", (int)transfer);
					
					if (transferTime == -1) { // only do this once for any planet, use period to shift
						transferTime = guiData.Sys->hohmannCalc(someSat->soiIdx, someSat->targStat->targetIdx,
							*guiData.simTime, synPeriod);

						if (synPeriod != -1 && synPeriod < 0)
							synPeriod *= -1;

						char tDate[80];
						double et = ((transferTime / 86400.0) + 2440587.5);
						et = (et - 2451545.0) * 86400.0; // JD to SPICE ET
						et2utc_c(et, "ISOC", 0, 80, (SpiceChar*)tDate);
						strncpy(transferDate, tDate, 10);
						transferDate[10] = '\0';

						transfer = 1;
					}

					
					int lastT = transfer;
					arrowToggle(transfer, true, false);
					ImGui::SameLine();
					if (ImGui::Button("Warp To") && transferTime != -1) {
						guiData.Sys->ArgClockSet(transferTime);
						currSat->stateTime = transferTime;
						if (copySat != nullptr) {
							copySat->stateTime = transferTime;
							newManDt = transferTime + (60 * 2);
						}
					}

					if (transfer != lastT) {
						transferTime += (transfer - lastT) * synPeriod;

						// hell yes i copied this code
						char tDate[80];
						double et = ((transferTime / 86400.0) + 2440587.5);
						et = (et - 2451545.0) * 86400.0; // JD to SPICE ET
						et2utc_c(et, "ISOC", 0, 80, (SpiceChar*)tDate);
						strncpy(transferDate, tDate, 10);
						transferDate[10] = '\0';
					}

					if (transfer == 0 && lastT > 0)
						transfer = -1;
					else if (transfer == 0 && lastT < 0)
						transfer = 1;

					
					
					if (transferTime != -1) {
						ImGui::Text("Transfer Date: %s", transferDate); 
						ImGui::Text("\tT%s", secToDay(*guiData.simTime - transferTime));
					}
					else {
						ImGui::Text("Transfer not found in 3 year window.");
					}

				}
			}
			ImGui::End();
		}

		if (ImGui::Button("Show Maneuvers")) {
			showMan = !showMan;
			manListCurr = 0;
		}

		// maneuver stats
		if (showMan) {
			ImGui::Combo("->", &manListCurr, manList, manListLen);
			if (currSat->maneuvers.size() > manListCurr) {
				ImGui::SameLine();
				if (ImGui::Button("Go-To")) {
					goToManTime = currSat->maneuvers[manListCurr].time;
					currSat->ArtSatUpdState(guiData.Sys->bodies, goToManTime, *guiData.tW, 1);
					currSat->lastEphTime = goToManTime;
				}

				ImGui::Text("%s", currSat->maneuvers[manListCurr].desc);
				ImGui::Text("deltaV: %.2f m/s", glm::length(currSat->maneuvers[manListCurr].newState.Vel - currSat->maneuvers[manListCurr].origState.Vel) * 1000);
				ImGui::Text("Burn in T%s", secToDay(currSat->maneuvers[manListCurr].time - *guiData.simTime));
			}
		}

		ImGui::SameLine();
		if (ImGui::Button("New Maneuver") && currSat->inTime) {
			if (newMan == true) { // if already open, reset & close
				delete copySat;
				copySat = nullptr;
				newMan = false;
				manDt = 0;
				manData = { 0, 0, 0 };
			}
			else {
				newMan = true;
				newManDt = *guiData.simTime + (60 * 2);
				copySat = new ArtSat(*currSat);
				copySat->isCopy = true;
			}

		}

		if (ImGui::Button("Save") && !newMan) {
			// System fxn to save a sat
			guiData.Sys->sat2Persist(*currSat, true);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextUnformatted("Saves satellite data into persistent memory \n (won't save if maneuvering)");
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete")) {
			guiData.Sys->sat2Persist(*currSat, false);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextUnformatted("Deletes satellite data from persistent memory if there");
			ImGui::EndTooltip();
		}


		if (newMan) { // maneuver logic

			// should not use static here

			// use the number of maneuvers after launch (total size - 1) to append the below string
			int manNum = currSat->maneuvers.size();
			char manNumStr[4];
			snprintf(manNumStr, (int)((ceil(log10(manNum)) + 1) * sizeof(char)), "%d", manNum);

			static char str0[30] = "maneuver "; // modify to tick up with amt of maneuvers
			if (strcmp(str0, "maneuver ") == 0)
				strcat(str0, manNumStr);
			ImGui::InputText("Name", str0, IM_ARRAYSIZE(str0));

			static char str1[30] = "";
			ImGui::InputText("Description", str1, IM_ARRAYSIZE(str1));

			ImGui::Checkbox("Fine Control", &fineCtrl);
			if (ImGui::IsItemHovered()) {
				ImGui::BeginTooltip();
				ImGui::TextUnformatted("Modify slider bounds for small adjustment");
				ImGui::EndTooltip();
			}
			ImGui::SameLine();
			ImGui::Checkbox("Retrograde", &retrograde);

			
			if (ImGui::Button("- Test Man -")) {
				manDt = 1801759782 - *(guiData.simTime);
				manData[0] = .597609997;
				manData[1] = -.00940000266;
				manData[2] = .00259999558;
			}
			


			manData[0] *= 1000; // have to move this above fineCtrl copy
			if (fineCtrl && fineCtrlData == glm::vec3{ 0, 0, 0 }) {
				fineCtrlData = manData;
			}
			else if (!fineCtrl && fineCtrlData != glm::vec3{ 0, 0, 0 }) {
				fineCtrlData = glm::vec3{ 0, 0, 0 };
			}


			ImGui::SliderFloat("Time", &manDt, 0, currSat->stat->orbitalPeriod, "%.f s");
			ImGui::SameLine();
			arrowToggle(manDt, true, fineCtrl);
			if (manDt < 0) manDt += currSat->stat->orbitalPeriod;



			if (fineCtrl) { 
				float lBnd = fineCtrlData[0] - 75; float uBnd = fineCtrlData[0] + 75;
				if (lBnd < 0)
					lBnd = 0;
				ImGui::SliderFloat("Velocity", &manData[0], lBnd, uBnd, "%.3f m/s");
			}
			else
				ImGui::SliderFloat("Velocity", &manData[0], 0, 10000, "%.2f m/s");
			ImGui::SameLine();
			arrowToggle(manData[0], true, fineCtrl);
			manData[0] /= 1000;




			if (fineCtrl) {
				float lBnd = fineCtrlData[1] - 0.2; float uBnd = fineCtrlData[1] + 0.2;
				if (lBnd < -glm::pi<float>() / 2)
					lBnd = -glm::pi<float>() / 2;
				else if (uBnd > glm::pi<float>() / 2)
					uBnd = glm::pi<float>() / 2;
				ImGui::SliderFloat("Pitch", &manData[1], lBnd, uBnd, "%.3f rad");
			}
			else
				ImGui::SliderFloat("Pitch", &manData[1], -glm::pi<float>() / 2, glm::pi<float>() / 2, "%.2f rad");
			ImGui::SameLine();
			arrowToggle(manData[1], false, fineCtrl);

			if (manData[1] > glm::pi<float>() / 2) manData[1] -= glm::pi<float>();
			if (manData[1] < -glm::pi<float>() / 2) manData[1] += glm::pi<float>();


			if (fineCtrl) {
				float lBnd = fineCtrlData[2] - 0.4; float uBnd = fineCtrlData[2] + 0.4;
				if (lBnd < -glm::pi<float>())
					lBnd = -glm::pi<float>();
				else if (uBnd > glm::pi<float>())
					uBnd = glm::pi<float>();
				ImGui::SliderFloat("Yaw", &manData[2], lBnd, uBnd, "%.3f rad");
			}
			else
				ImGui::SliderFloat("Yaw", &manData[2], -glm::pi<float>(), glm::pi<float>(), "%.2f rad");
			ImGui::SameLine();
			arrowToggle(manData[2], false, fineCtrl);
			if (manData[2] > glm::pi<float>()) manData[2] -= 2 * glm::pi<float>();
			if (manData[2] < -glm::pi<float>()) manData[2] += 2 * glm::pi<float>();

			// handle retrograde
			if (retrograde && manData[0] > 0)
				manData[0] = -manData[0];

			// if maneuver changed do new calculation
			if (oldManDt != manDt || oldManData != manData) {
				// handle thread atomic bool
				manStopBool->store(true);
				while (guiData.Sys->maneuverThread.valid()) { // make sure thread is dead
					if (guiData.Sys->maneuverThread.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
						guiData.Sys->maneuverThread.get();
					}
				}
				manStopBool->store(false);

				// transplant target, remake copySat
				targStats copyTS; copyTS.targetIdx = -1;
				if (copySat->targStat != nullptr)
					copyTS = *(copySat->targStat);

				delete copySat;
				copySat = new ArtSat(*currSat);

				if (copyTS.targetIdx != -1)
					copySat->targStat = new targStats(copyTS);
				copySat->isCopy = true;
				// solve new maneuver
				copySat->ArtSatManeuver(manData, guiData.Sys->bodies, newManDt + manDt, str0, str1);
			}
			if (guiData.Sys->maneuverThread.valid()) { // ********************************************************************* fillBuff capture for orbit draw
				// fill up temp buffer for early render
				if (guiData.Sys->maneuverThread.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
					if (!(copySat->soi_ing->load()) && !(copySat->escaping)) // checking if going to other soi (moon) or has escaped -- what does soi_ing have to do w/ fillBuff here?
						copySat->fillBuff(0); // bounded orbit
					else
						copySat->fillBuff(1); // escape check / escape
				}
				else {
					// or if done processing empty thread
					guiData.Sys->maneuverThread.get();

					// handle target closest approach
					if (copySat->targStat != nullptr) {
						copySat->chartApproach(guiData.Sys->bodies, copySat->targStat->targetIdx);
						copySat->mk1->color = copySat->lineColor;
						copySat->mk2->color = copySat->lineColor;
					}
					copySat->threadStop = nullptr;
				}
			}
			// render and update ( add modifier to change orbit color, fade, + update and render actual sat at same time )
			copySat->ArtSatUpdState(guiData.Sys->bodies, newManDt + manDt, *guiData.tW, 1); // right time????
			
			oldManDt = manDt;
			oldManData = manData;
			// handle retrograde
			if (manData[0] < 0)
				manData[0] = -manData[0];
			// save maneuver
			if (ImGui::Button("Save Maneuver")) {
				if (strcmp(str1, "\0") == 0) {
					strcpy(str1, "?\0");
				}
				currSat->maneuvers.push_back({ (copySat->maneuvers[copySat->maneuvers.size() - 1].origState),
					(copySat->maneuvers[copySat->maneuvers.size() - 1].newState), newManDt + manDt, str0, str1, copySat->soiIdx });
				delete copySat;
				copySat = nullptr;
				manDt = 0;
				oldManDt = manDt;
				manData = { 0, 0, 0 };
				oldManData = manData;
				newMan = false;
			}
		}
		else if (manDt != 0 || oldManDt != 0) {
			manDt = 0;
			oldManDt = manDt;
			manData = { 0, 0, 0 };
			oldManData = manData;
		}
	}






	
	if (ImGui::Button("New Artificial Satellite")) {
		if (newArtSat) {}
		else {
			newArtSat = true;
			sat = new ArtSat();
			pv = new pvUnit;
			sat->sysThread = &guiData.Sys->maneuverThread;
			sat->threadStop = &guiData.Sys->satManStop;
			sat->mtxSat = &guiData.Sys->mtxSat;
		}
	}
	if (newArtSat) { // new satellite logic

		static char str0[30] = "Untitled Spacecraft 1"; // word input, also use to display info
		ImGui::InputText("Name", str0, IM_ARRAYSIZE(str0));
		
		int dummy = (int)initPos[0];
		ImGui::SliderInt("r", &dummy, 300, 10000, "%d km", ImGuiInputTextFlags_EnterReturnsTrue); // when r changes, vMag should change to keep the same orbital energy over some threshold
		initPos[0] = dummy;
		ImGui::SameLine();
		arrowToggle(initPos[0], true, fineCtrl);
		
		ImGui::SliderFloat("theta", &initPos[1], 0, glm::pi<float>(), "%.2f rad", ImGuiInputTextFlags_EnterReturnsTrue);
		ImGui::SameLine();
		arrowToggle(initPos[1], false, fineCtrl);
		if (initPos[1] < 0) initPos[1] += glm::pi<float>();

		ImGui::SliderFloat("phi", &initPos[2], 0, 2 * glm::pi<float>(), "%.2f rad", ImGuiInputTextFlags_EnterReturnsTrue);
		ImGui::SameLine();
		arrowToggle(initPos[2], false, fineCtrl);
		if (initPos[2] < 0) initPos[2] += 2 * glm::pi<float>();

		ImGui::SliderFloat("Velocity", &initVel[0], 0, 12, "%.2f m/s", ImGuiInputTextFlags_EnterReturnsTrue);
		ImGui::SameLine();
		arrowToggle(initVel[0], false, fineCtrl);
		if (initVel[0] < 0) initVel[0] += 12;
		
		ImGui::SliderFloat("Pitch", &initVel[1], 0, glm::pi<float>(), "%.2f rad", ImGuiInputTextFlags_EnterReturnsTrue);
		ImGui::SameLine();
		arrowToggle(initVel[1], false, fineCtrl);
		if (initVel[1] < 0) initVel[1] += glm::pi<float>();

		ImGui::SliderFloat("Yaw", &initVel[2], 0, 2 * glm::pi<float>(), "%.2f rad", ImGuiInputTextFlags_EnterReturnsTrue);
		ImGui::SameLine();
		arrowToggle(initVel[2], false, fineCtrl);
		if (initVel[2] < 0) initVel[2] += 2 * glm::pi<float>();

		// init pv
		glm::vec3 cartDummy = sphToCart(initPos + glm::vec3{guiData.Sys->bodies[3]->realRadius, 0, 0});
		pv->Pos = glm::dvec3{ cartDummy.z, cartDummy.x, cartDummy.y};

		glm::vec3 relToPos = { 0, initPos[1] - glm::pi<float>() / 2, initPos[2] };
		cartDummy = sphToCart(initVel + relToPos);
		pv->Vel = glm::dvec3{ cartDummy.z, cartDummy.x, cartDummy.y };

		// process pv

		sat->ArtSatPlan(*pv, *(guiData.simTime), 3, guiData.Sys->bodies);

		if (guiData.Sys->maneuverThread.valid()) {
			// fill up temp buffer for early render
			if (guiData.Sys->maneuverThread.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
				if (!(sat->soi_ing->load()))
					sat->fillBuff(0);
				else
					sat->fillBuff(1);
			}
			else {
				// or if done processing empty thread
				guiData.Sys->maneuverThread.get();

				// handle target closest approach
				if (sat->targStat != nullptr) {
					sat->chartApproach(guiData.Sys->bodies, sat->targStat->targetIdx);
				}
			}
		}


		sat->ArtSatUpdState(guiData.Sys->bodies, *guiData.simTime, *guiData.tW, -1);

		// reset art sat parameters to default values in each, delete pvUnit as well
		if (ImGui::Button("Save")) {
			bool repeatName = false;
			for (auto& someSat : guiData.Sys->artSats) {
				if (someSat.name == str0) repeatName = true;
			}
			if (!repeatName) {
				sat->name = str0;
				guiData.Sys->artSats.insert(guiData.Sys->artSats.begin(), *sat);
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

void arrowToggle(float& value, bool isInt, bool fineCtrl) {
	float inc = 0;
	if (isInt)
		inc = 1;
	else
		inc = 0.01;
	if (fineCtrl)
		inc /= 50;
	ImGui::PushItemFlag(ImGuiItemFlags_ButtonRepeat, true);
	if (ImGui::ArrowButton(std::to_string(rand()).c_str(), ImGuiDir_Left)) { value -= inc; }
	ImGui::SameLine();
	if (ImGui::ArrowButton(std::to_string(rand()).c_str(), ImGuiDir_Right)) { value += inc; } // when tw value gets bigger, this moves to compensate, should fix pos
	ImGui::PopItemFlag();
}

// configured for T -/+ display
char* secToDay(double dt) {
	// catch erroneous dt's
	if (abs(dt) > INT_MAX) {
		char res[10] = "N/A";
		return res;
	}
	
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