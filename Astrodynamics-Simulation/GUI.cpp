#include "GUI.h"

GUI::GUI(GLFWwindow* window, GUIData guiData) {

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	bodyNames = new const char*[guiData.bodies.size()]; // delete this in deconstructor
	int counter = 0;
	for (auto body : guiData.bodies) {
		if (!body->areRings) {
			bodyNames[counter] = body->name;
			counter++;
		}
	}
	bodyNamesLen = counter;

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

	// add kill time warp button
	if (ImGui::Button("Kill TimeWarp"))
		*(guiData.tW) = 15;

	ImGui::SeparatorText("FOCUS CONTROL");
	if (ImGui::Button("Focus Mode")) // wish i could change this str from focus to free when clicked to toggle
		guiData.camera->focusMode = !(guiData.camera->focusMode);

	// focuslist should only show when focusmode is true
	if (guiData.camera->focusMode) {
		ImGui::Combo("Body Select", &(guiData.camera->focusBody), bodyNames, bodyNamesLen);
	}

	ImGui::SeparatorText("ART SAT CONTROL");

	
	if (ImGui::Button("New Artificial Satellite")) {
		newArtSat = true;
		sat = new ArtSat();
		pv = new pvUnit;
	}
	if (newArtSat) {
		// should init positions in spherical coordinates and velocity in cartesian
		// transfer to cartesian and plug into a pvUnit for Plan
		
		// could stop time while newArtSat is true
		/*
		cool orbit
		14908.5 0 -2916.72
		-1.04933 2.29283 -5.33403
		*/
		//************************ position sliders dont work (theta / phi)
		ImGui::SliderInt("r", &r, 300, 10000, "ratio = %10");
		//ImGui::SameLine();
		ImGui::SliderFloat("theta", &theta, 0, glm::pi<float>(), "ratio = %.2f");
		//ImGui::SameLine();
		ImGui::SliderFloat("phi", &phi, 0, 2 * glm::pi<float>(), "ratio = %.2f");

		ImGui::SliderFloat("Velocity", &vMag, 1, 9, "ratio = %.2f");
		//ImGui::SameLine();
		ImGui::SliderFloat("Pitch", &vTheta, 0, glm::pi<float>(), "ratio = %.2f");
		//ImGui::SameLine();
		ImGui::SliderFloat("Yaw", &vPhi, 0, 2 * glm::pi<float>(), "ratio = %.2f");
		
		glm::vec3 cartDummy = sphToCart(glm::vec3(r + guiData.bodies[3]->realRadius, theta, phi));
		pv->Pos = glm::vec3{ cartDummy.z, cartDummy.x, cartDummy.y};
		cartDummy = sphToCart(glm::vec3(vMag, vTheta, vPhi));
		pv->Vel = glm::vec3{ cartDummy.z, cartDummy.x, cartDummy.y };
		sat->ArtSatPlan(*pv, *(guiData.simTime), 3, guiData.bodies);
		

		// reset art sat parameters to default values in each, delete pvUnit as well
		if (ImGui::Button("Save")) {
			sat->ArtSatSave();
			newArtSat = false;
			
		}
		ImGui::SameLine();
		if (ImGui::Button("Discard")) {
			sat->deleteArtSat();
			newArtSat = false;
		}
		if (ImGui::Button("Print Orbit")) {
			std::cout << pv->Pos.x << " " << pv->Pos.y << " " << pv->Pos.z << '\n' << pv->Vel.x << " " << pv->Vel.y << " " << pv->Vel.z << '\n';
		}
	}

	/*
	// drop down
	const char* items[] = { "AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "FFFF", "GGGG", "HHHH", "IIIIIII", "JJJJ", "KKKKKKK" };
			static int item_current = 0;
			ImGui::Combo("combo", &item_current, items, IM_ARRAYSIZE(items));


	ImGui::SeparatorText("ABOUT THIS DEMO:"); // -- words ---------

	// Arrow buttons with Repeater
	//IMGUI_DEMO_MARKER("Widgets/Basic/Buttons (Repeating)");
	static int counter = 0;
	float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
	ImGui::PushItemFlag(ImGuiItemFlags_ButtonRepeat, true);
	if (ImGui::ArrowButton("##left", ImGuiDir_Left)) { counter--; }
	ImGui::SameLine(0.0f, spacing);
	if (ImGui::ArrowButton("##right", ImGuiDir_Right)) { counter++; }
	ImGui::PopItemFlag();
	ImGui::SameLine();
	ImGui::Text("%d", counter);

	ImGui::Button("Tooltip");
	ImGui::SetItemTooltip("I am a tooltip"); // floating box when hover

	//IMGUI_DEMO_MARKER("Widgets/Basic/Button"); // button click, print line beside
	static int clicked = 0;
	if (ImGui::Button("Button"))
		clicked++;
	if (clicked & 1)
	{
		ImGui::SameLine();
		ImGui::Text("Thanks for clicking me!");
	}

	static int i0 = 123; // numerical input with -/+ buttons
	ImGui::InputInt("input int", &i0);

	static char str0[128] = "Hello, world!"; // word input, also use to display info
	ImGui::InputText("input text", str0, IM_ARRAYSIZE(str0));
	*/

	ImGui::End();

}

void GUI::guiLoopEnd(GUIData guiData) {
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	if (newArtSat) {
		sat->ArtSatRender(guiData.camera);
	}
}

void GUI::guiDestroy() {
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}