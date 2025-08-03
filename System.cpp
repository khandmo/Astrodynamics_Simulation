#include "System.h"
#include <cstring>

//double distanceFind(glm::vec3 state1, glm::vec3 state2);
void stateVecToSim(char* vec, state& someState);
void sat2CSV(ArtSat& sat, char* buff, int& buffSize);
void csv2Sat(ArtSat& sat, char* buff, std::vector<Mesh*>* bodies);
char* chrono2SpiceStr(const char* input);

System::System() {
	// init SPICE kernels
	furnsh_c("spice_kernels/de441_part-2.bsp"); // for pos/vel of bodies (baryID)
	furnsh_c("spice_kernels/de441_part-1.bsp");
	furnsh_c("spice_kernels/pck00011.tpc.txt"); // for axial orientation (spiceID)
	furnsh_c("spice_kernels/naif0012.tls.txt");
	furnsh_c("spice_kernels/jup365.bsp"); // jupiter main satellite positions
	furnsh_c("spice_kernels/sat441.bsp"); // saturn main satellite positions
	furnsh_c("spice_kernels/ura182.bsp"); // uranus main satellite positions
	furnsh_c("spice_kernels/nep095.bsp"); // neptune main satellite positions
	erract_c("SET", 0, (SpiceChar*)"RETURN");

	SystemTime();

	// initialize all solar system bodies, a body's gravitational source must be initialized before that body
	// initialize radius as true radius in km divided by a factor of 6100/4550000000 to be consistent with orbital distances
	bodiesActual.push_back(initBody("Sun", "Textures/SQmercury.jpg", 1.989f * pow(10, 10), 696340.0f, 0.0f, 0.0f, true, false, "10", 10, 10, 0)); // CODE FIX FOR soiID if it is -1 to orbit the center of the universe or stay still
	bodiesActual.push_back(initBody("Mercury", "Textures/SQmercury.jpg", 3.3011f * pow(10, 3), 2439.7f, 0.0f, 2, false, false, "10", 1, 199, 88));
	bodiesActual.push_back(initBody("Venus", "Textures/SQvenus.jpg", 4.8675f * pow(10, 4), 6051.8f, 0.0f, 3, false, false, "10", 2, 299, 225));
	bodiesActual.push_back(initBody("Earth", "Textures/earth4096.jpg", 5.97237f * pow(10, 4), 6371.0f, 0.0f, 23.5, false, false, "10", 3, 399, 365));
	bodiesActual.push_back(initBody("Moon", "Textures/moon4096.jpg", 7.342f * pow(10, 2), 1737.4f, 0.0f, 1.5, false, false, "399", 301, 301, 27));

	
	bodiesActual.push_back(initBody("Mars", "Textures/SQmars.jpg", 6.4171f * pow(10, 3), 3389.5f, 0.0f, 25, false, false, "10", 4, 499, 687));
	bodiesActual.push_back(initBody("Jupiter", "Textures/SQjupiter.jpg", 1.8982f * pow(10, 7), 69911.0f, 0.0f, 3, false, false, "10", 5, 599, 4331));
	bodiesActual.push_back(initBody("Io", "Textures/SQgrey.jpg", 8.93 * pow(10, 2), 1560.0f, 0.0f, 0, false, false, "5", 5, 501, 1.7691f));
	bodiesActual.push_back(initBody("Europa", "Textures/SQgrey.jpg", 4.7998 * pow(10, 2), 1560.8f, 0.0f, 0, false, false, "5", 5, 502, 3.5512f));
	bodiesActual.push_back(initBody("Ganymede", "Textures/SQgrey.jpg", 1.4819 * pow(10, 3), 2634.1f, 0.0f, 0, false, false, "5", 5, 503, 7.1546f));
	bodiesActual.push_back(initBody("Callisto", "Textures/SQgrey.jpg", 1.0759 * pow(10, 3), 2410.3f, 0.0f, 0, false, false, "5", 5, 504, 16.689f));

	bodiesActual.push_back(initBody("Saturn", "Textures/SQsaturn.jpg", 5.6834f * pow(10, 6), 58232.0f, 0.0f, 26.73, false, false, "10", 6, 699, 10759));
	bodiesActual.push_back(initBody("saturnRings", "Textures/SQsaturnRings.jpg", 0.0f, 75000.0f, 140000.0f, 26.73, false, true, "10", 6, 699, 10759));
	bodiesActual.push_back(initBody("Mimas", "Textures/SQgrey.jpg", 3.749 * pow(10, -1), 396.0f, 0.0f, 0, false, false, "6", 6, 601, 0.942f));
	bodiesActual.push_back(initBody("Enceladus", "Textures/SQgrey.jpg", 1.08 * pow(10, 0), 257.0f, 0.0f, 0, false, false, "6", 6, 602, 1.37f));
	bodiesActual.push_back(initBody("Tethys", "Textures/SQgrey.jpg", 6.176 * pow(10, 0), 538.0f, 0.0f, 0, false, false, "6", 6, 603, 1.888f));
	bodiesActual.push_back(initBody("Dione", "Textures/SQgrey.jpg", 1.096 * pow(10, 1), 563.0f, 0.0f, 0, false, false, "6", 6, 604, 2.737f));
	bodiesActual.push_back(initBody("Rhea", "Textures/SQgrey.jpg", 2.31 * pow(10, 1), 765.0f, 0.0f, 0, false, false, "6", 6, 605, 4.518f));
	bodiesActual.push_back(initBody("Titan", "Textures/SQgrey.jpg", 1.3455 * pow(10, 3), 2575.0f, 0.0f, 0, false, false, "6", 6, 606, 15.945f));
	bodiesActual.push_back(initBody("Iapetus", "Textures/SQgrey.jpg", 1.81 * pow(10, 1), 746.0f, 0.0f, 0, false, false, "6", 6, 608, 79.32f));

	bodiesActual.push_back(initBody("Uranus", "Textures/SQuranus.jpg", 8.681 * pow(10, 5), 25362.0f, 0.0f, 97.7, false, false, "10", 7, 799, 30689));
	bodiesActual.push_back(initBody("Titania", "Textures/SQgrey.jpg", 3.42 * pow(10, 1), 788.9f, 0.0f, 0, false, false, "7", 7, 703, 8.7f));
	bodiesActual.push_back(initBody("Oberon", "Textures/SQgrey.jpg", 2.88 * pow(10, 1), 761.4f, 0.0f, 0, false, false, "7", 7, 704, 13.46f));
	bodiesActual.push_back(initBody("Umbriel", "Textures/SQgrey.jpg", 1.22 * pow(10, 1), 584.7f, 0.0f, 0, false, false, "7", 7, 702, 4.14f));
	bodiesActual.push_back(initBody("Ariel", "Textures/SQgrey.jpg", 1.29 * pow(10, 1), 581.1f, 0.0f, 0, false, false, "7", 7, 701, 2.52f));
	bodiesActual.push_back(initBody("Miranda", "Textures/SQgrey.jpg", 6.6 * pow(10, -1), 240.0f, 0.0f, 0, false, false, "7", 7, 705, 1.41f));
	
	bodiesActual.push_back(initBody("Neptune", "Textures/SQneptune.jpg", 1.02413 * pow(10, 6), 24622.0f, 0.0f, 28, false, false, "10", 8, 899, 60182));
	bodiesActual.push_back(initBody("Triton", "Textures/SQgrey.jpg", 2.14 * pow(10, 2), 1353.0f, 0.0f, 0, false, false, "8", 8, 801, 5.87f));
	bodiesActual.push_back(initBody("Proteus", "Textures/SQgrey.jpg", 4.4 * pow(10, -2), 210.0f, 0.0f, 0, false, false, "8", 8, 808, 1.12f));
	//bodiesActual.push_back(initBody("Nereid", "Textures/SQgrey.jpg", 3.1 * pow(10, -3), 170.0f, 0.0f, 0, false, false, "8", 8, 802, 360.136f)); no axial data, will tear up program
	

	// transplant bodies addresses
	for (int i = 0; i < bodiesActual.size(); i++) {
		bodies.push_back(&bodiesActual[i]);
		bodies[i]->spiceMtx = &mtx;
		bodies[i]->idx_int = i;
		if (bodies[i]->isLightSource) {
			lightBodies.push_back(bodies[i]);
		}
		else {
			dullBodies.push_back(bodies[i]);
		}
		if (i != 0) {
			bodies[i]->gravSource = &bodiesActual[std::stoi(bodies[i]->soiIdx)];

			// assign soi radii
			glm::dvec3 dPos = *bodies[i]->Pos, rPos;
			invStateChange(&dPos, &rPos);
			if (bodies[i]->gravSource != nullptr)
				bodies[i]->soiRadius = glm::length(dPos) * pow((bodies[i]->mass / bodies[i]->gravSource->mass), 2.0 / 5.0);
		}
		else
			bodies[0]->soiRadius = INT_MAX;
	}

	// init art sats
	// program to take artSats and name of ephemeris data, process and add mission + maneuver to persistent memory
	initPersistSats("persistent_sats.txt");
	initSat("Artemis 1", "hr_artemis_1.txt");

	// time memory save
	sysTime = *(time_block*) malloc(sizeof(time_block));
	simTime = *(time_block*) malloc(sizeof(time_block));

	// set shaders
	shaderSet();
}

Mesh System::initBody(const char* name, const char* texFilePath, float mass, float radius, float outerRadius, float axialTilt, bool isLight, bool areRings, const char* soiID, int baryID, int spiceID, float orbPeriod) {
	// init texture
	Texture tex[] = {Texture(texFilePath, "diffuse", 0, GL_RGB, GL_UNSIGNED_BYTE)};
	std::vector <Texture> objTex(tex, tex + sizeof(tex) / sizeof(Texture));

	// init object
	Object* obj = new Object; // ******** 891.33 KB heap

	if (!areRings) {
		obj->Box(radius * LENGTH_SCALE);
	}
	else {
		obj->Rings(radius * LENGTH_SCALE, outerRadius * LENGTH_SCALE);;
	}
	
	Shader shader = dS;
	if (isLight) {
		shader = lS;
	}

	// masses are all / 10^20 for the sake of transportation

	// if soiID is 10 then soiIdx is 0, else it's a moon and check last planet in bodies !!!!!!!!!!!!!!!!!!!!!!!!!!
	char soiIdx[3];
	if (!std::strcmp(soiID, "10\0")) {
		strcpy(soiIdx, "0\0");
	}
	else {
		for (int i = bodiesActual.size() - 1; i > 0; i--) {
			if (!bodiesActual[i].isMoon && !bodiesActual[i].areRings) {
				strcpy(soiIdx, std::to_string(i).c_str());
				break;
			}
		}
	}


	 // ********** mesh init has 1,806 KB heap
	Mesh body(name, obj->vertices, obj->indices, objTex, radius, mass, isLight, areRings, &shader, soiID, soiIdx, baryID, spiceID, sysTime.time_in_sec, orbPeriod); // velocity will be updated with SPICE integration
	// Set Properties	
	body.AxialTilt(axialTilt);
	bodyRadii.push_back(radius * LENGTH_SCALE);

	return body;
}

void System::initSat(const char* name, const char* eph) {

	// check if name is already in ArtSat list (initialized from persistent memory)
	for (int i = 0; i < artSats.size(); i++) {
		if (strstr(artSats[i].name, name) != nullptr) {
			return;
		}
	}

	std::cout << "adding new satellite to simulation...";

	// hold maneuvers
	char* mane[100];

	// parse text file
	std::fstream dataStream(eph, std::fstream::in); // can read and write
	if (!dataStream) {
		std::cout << "File failed to open / create\n";
		dataStream.clear();
		return;
	}

	int maneIdx = 0;
	while (1) {
		// get maneuver information
		char* buff = new char[300];
		dataStream.getline(buff, 300);

		if (buff[0] == buff[1] && buff[1] == buff[2] && buff[2] == '*') { // *** terminates maneuver information
			break;
		}
		mane[maneIdx++] = buff;
	}

	// parse and save launch information
	char buff[300] = "";
	char buffBack[300] = "";
	state someState;
	dataStream.getline(buff, 300);
	stateVecToSim(buff, someState);

	ArtSat* sat = new ArtSat();
	sat->sysThread = &maneuverThread;
	sat->threadStop = &satManStop;
	sat->mtxSat = &mtxSat;
	double testTime = someState.time;
	sat->ArtSatPlan({ someState.pos, someState.vel }, testTime, 3, bodies);
	sat->name = name;

	// parse and save maneuvers, end state
	int maneIdx2 = 1;
	while (1) {
		dataStream.getline(buff, 300);

		if (buff[0] == buff[1] && buff[1] == buff[2] && buff[2] == '*' || maneIdx2 > maneIdx - 1) { // *** terminates maneuver information
			break;
		}
		//maneuver importing below, buff for date and time of data point
		char dateBuff[300], timeBuff[300];
		strncpy(dateBuff, buff + 24, sizeof(buff) - 24);
		strncpy(timeBuff, dateBuff + 12, sizeof(buff) - 35);
		dateBuff[12] = '\0';
		timeBuff[2] = '\0';

		char* man_date = strstr(mane[maneIdx2], dateBuff);
		if (man_date != nullptr) { // if data line is date of a maneuver
			
			char* man_time = man_date + 12;
			man_time[2] = '\0'; // hold the time of maneuver

			int diff = std::stoi(timeBuff) - std::stoi(man_time);
			if (diff > 0 && diff < 6 || std::stoi(timeBuff) > 18){// ****** assume ephermis data taken 6 hours apart
				
				char man_name[300] = "";
				strncpy(man_name, mane[maneIdx2] + 2, strlen(mane[maneIdx2]) - 1);
				size_t end = strcspn(man_name, " ");
				man_name[end] = '\0';

				state aState, bState;
				stateVecToSim(buff, bState);
				stateVecToSim(buffBack, aState);
				sat->solveManeuver(bodies, man_name, { aState.pos, aState.vel }, { bState.pos, bState.vel }, aState.time, bState.time);			
				
				maneIdx2++;
			}
		}
		strncpy(buffBack, buff, strlen(buff) + 1);
		buffBack[(strlen(buff))] = '\0';
		
	}
	char* token = strtok(buffBack, ";");
	double endTime = atof(token);
	endTime = (endTime - 2440587.5) * 86400.0; // to UNIX
	// add to artSat, check time at update

	dataStream.clear();
	dataStream.close();


	sat->lastEphTime = endTime;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	artSats.push_back(*sat);


	// free all held maneuver strings
	while (maneIdx != 0) {
		delete mane[--maneIdx];
	}

	std::cout << "new satellite " << sat->name << " added\n";

	// add sat to end of persistent txt (w/ new line at end)
	std::fstream dataStream2(persistent_memory_address, std::fstream::out | std::fstream::app); // should just read
	if (!dataStream2) {
		std::cout << "File " << persistent_memory_address << " failed to open / create\n";
		dataStream2.clear();
		return;
	}
	
	char buff2[10000] = "";
	int buff2Size = 0;
	sat2CSV(*sat, buff2, buff2Size);

	dataStream2.seekp(0, std::fstream::end);
	dataStream2.write(buff2, buff2Size);
	dataStream2.write("\n", 1);
	dataStream2.clear();
	dataStream2.close();
	delete sat;

}

void System::initPersistSats(const char* file) {
	char* fileHold = (char*)malloc(strlen(file) + 1);
	strcpy(fileHold, file);
	persistent_memory_address = fileHold;

	std::fstream dataStream(file, std::fstream::in); // should just read
	if (!dataStream) {
		std::cout << "File " << file << " failed to open / create\n";
		dataStream.clear();
		return;
	}
	
	char buff[10000] = "";
	dataStream.getline(buff, 10000);
	while (!dataStream.eof()) {
		ArtSat* sat = new ArtSat;
		sat->sysThread = &maneuverThread;
		sat->threadStop = &satManStop;
		sat->mtxSat = &mtxSat;
		csv2Sat(*sat, buff, &bodies);
		artSats.push_back(*sat);
		dataStream.getline(buff, 10000);


		while (maneuverThread.valid()) { // make sure thread is dead so info isn't exchanged
			if (maneuverThread.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
				maneuverThread.get();
			}
		}
		delete sat;
	}


	dataStream.clear();
	dataStream.close();

	// sat info is wiped after this function exits
}

bool System::sat2Persist(ArtSat sat, bool add) {
	std::fstream dataStream(persistent_memory_address, std::fstream::in | std::fstream::out);
	if (!dataStream) {
		std::cout << "File " << persistent_memory_address << " failed to open / create\n";
		dataStream.clear();
		return false;
	}
	std::fstream tempStream("temp.txt", std::fstream::out | std::fstream::app);
	if (!tempStream) {
		std::cout << "Temp file failed to create\n";
		tempStream.clear();
		return false;
	}

	bool jobDone = false;

	// search file
	char buff[10000] = "";
	dataStream.getline(buff, 10000);
	while (!dataStream.eof()) {
		char* satInFile = strstr(buff, sat.name);

		if (satInFile != NULL) { // if sat found
			jobDone = true;
			dataStream.getline(buff, 10000);
			continue;
		}

		tempStream.write(buff, strlen(buff)); // copy data into temp (skip if sat found)
		tempStream.write("\n", 1);
		dataStream.getline(buff, 10000);
	}

	dataStream.clear();
	dataStream.close();

	if (add) { // add sat to end of tempStream
		char buff2[10000] = "";
		int buff2Size = 0;
		sat2CSV(sat, buff2, buff2Size);

		tempStream.seekp(0, std::fstream::end);
		tempStream.write(buff2, buff2Size);
		tempStream.write("\n", 1);
		jobDone = true;
	}

	// if sub just delete, if ad
	tempStream.clear();
	tempStream.close();
	// replace file w/ temp file
	std::remove(persistent_memory_address);
	std::rename("temp.txt", persistent_memory_address);

	return jobDone;
}

void System::ArtSatHandle(Camera* camera, double dt, int &tW) {
	// if time warp over certain amount, run this in thread
	// push estimate positions to screen until time warp is brought back down and computations can complete 
	// use nodes for estimates, refresh traj if all out of nodes
	// can assume fit local nodes to low poly estimates to estimate position for visual purposes


	for (int i = 0; i < artSats.size(); i++) {
		// if sat has EOM
		if (artSats[i].mtxSat == nullptr)
			artSats[i].mtxSat = &mtxSat;

		artSats[i].ArtSatRender(camera, *(bodies[0]));
		artSats[i].ArtSatUpdState(bodies, dt, tW,  0);		
		if (i < satPos.size())
			satPos[i] = &artSats[i].simPos;
		else
			satPos.push_back(&artSats[i].simPos);
	}
}

void System::updateBodyState() { // System holds it's own positions, gets updated from Mesh --- System triggers Move and calls for velocity from body
	// update vector of bodyPos in glm vec3 format
	for (int i = 0; i < bodies.size(); i++) {
		if (bodyPos.size() >= i + 1)
			bodyPos[i] = (bodies[i]->Pos);
		else
			bodyPos.push_back((bodies[i]->Pos));

		
		Object obj;
		bool change = false;
		float apparent_size = abs(glm::length(camera->Position - *bodyPos[i])) / bodies[i]->radius;
		if (apparent_size < 600) {
			if (bodies[i]->vertices.size() < 50) { // if box and shouldn't be
				change = true;
				obj.Sphere(bodies[i]->radius);
			}
		}
		else {
			if (bodies[i]->vertices.size() > 50) { // if sphere and shouldn't be
				change = true;
				obj.Box(bodies[i]->radius);
			}
		}
		if (change && !bodies[i]->areRings) {
			bodies[i]->vertices = obj.vertices;
			bodies[i]->indices = obj.indices;

			bodies[i]->VAO.Bind();
			bodies[i]->my_EBO->Update(bodies[i]->indices);
			bodies[i]->my_VBO->Update(bodies[i]->vertices);			

			bodies[i]->VAO.LinkAttrib(*bodies[i]->my_VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);

			bodies[i]->VAO.Unbind();
			bodies[i]->my_VBO->Unbind();
			bodies[i]->my_EBO->Unbind();
		}
	}

	// for each body check apparent size from camera distance and radius, if above some amount make it a sphere, if below make it a box
}

void System::shaderSet() {
	for (auto body : lightBodies) { // sets emissions for all light bodies
		body->emissionShader();
	}
	for (auto body : dullBodies) { // sets shaders for all dull bodies in relation to light bodies
		for (auto lights : lightBodies) {
			body->dullShader(*lights);
		}
	}
}

void System::orbLineHandle(glm::vec3 cameraPos) {
	// handles distance related phasing of orbital lines

	double distPlanet = INT_MAX, distMoon = INT_MAX, dist;
	int planetIdx, moonIdx;
	for (int i = 0; i < dullBodies.size(); i++) {
		dist = distanceFind(*(dullBodies[i]->Pos), cameraPos);
		if (dist < distPlanet) {
			if (dullBodies[i]->isMoon) {
				distMoon = dist;
				moonIdx = i;
			}
			else {
				distPlanet = dist;
				planetIdx = i;
			}
		}
	}

	// HARD CUT OFF ELSEWHERE OF MOON ORBIT SHOW
	float planetDistLowBound = 12, planetDistHighBound = bodies[0]->refinedRadius, planetDistLowLowBound = 5.6f;
	float moonDistLowBound = 0.32f, moonDistHighBound = 1.6f; // *********** should use radius for these to scale, should make opacity calc helper fxn
	float brightColor = 0.5f;
	float dimColor = 0.1f;


	if (distPlanet > planetDistHighBound) { // far from any bodies
		for (auto body : dullBodies) {
			(body->lineColor).w = brightColor;
		}
	}
	else if (distPlanet > planetDistLowBound) { // function scale light color change, mid range
		
		float scale = planetDistHighBound - planetDistLowBound;
		float scaleAmt = distPlanet - planetDistLowBound;
		float opacityValue = ((scaleAmt / scale) * (brightColor - dimColor)) + dimColor;

		for (auto body : dullBodies) {
			(body->lineColor).w = opacityValue;
			if (body->isMoon) {
				(body->lineColor).w = brightColor - opacityValue;
			}
		}

	}
	else if (distPlanet < planetDistLowBound) { // very close to a body
		float scale = planetDistLowBound - planetDistLowLowBound;
		float scaleAmt = distPlanet - planetDistLowLowBound;
		float opacityValue = ((scaleAmt / scale) * dimColor);

		for (auto body : dullBodies) {
			if (opacityValue > 0) (body->lineColor).w = opacityValue;
			else (body->lineColor).w = 0;
			
			if (body->isMoon) (body->lineColor).w = brightColor;
			
			
		}
	}
	if (distMoon < moonDistHighBound && distMoon > moonDistLowBound) {
		float scale = moonDistHighBound - moonDistLowBound;
		float scaleAmt = distMoon - moonDistLowBound;
		float opacityValue = ((scaleAmt / scale) * brightColor);
		dullBodies[moonIdx]->lineColor.w = opacityValue;
	}
	else if (distMoon < moonDistLowBound) {
		dullBodies[moonIdx]->lineColor.w = 0;
	}
}

double System::hohmannCalc(int b1, int b2, double dt, double& synT) {
	if (bodies[b1]->isMoon || bodies[b2]->isMoon || b1 == 0 || b2 == 0)
		return -1;

	// assume circular orbits eg. r1 and r2 are fixed no matter the dt
	// compute transfer angle
	double mu = 6.67430 * pow(10, -20) * bodies[0]->mass;

	double tTime, r1, r2, w1, w2, phase;

	glm::dvec3 r1V = bodies[b1]->getPV(dt, false).Pos;
	glm::dvec3 r2V = bodies[b2]->getPV(dt, false).Pos;

	r1 = glm::length(r1V);
	r2 = glm::length(r2V);

	tTime = glm::pi<double>() * sqrt(pow((r1 + r2) / 2, 3) / mu);

	w1 = sqrt(mu / pow(r1, 3)); w2 = sqrt(mu / pow(r2, 3));

	phase = (w2 * tTime);
	phase = glm::pi<double>() - std::fmod(phase, 2 * glm::pi<double>());

	// compute current phase
	double currPhase = acos(glm::dot(r1V, r2V) / (r1 * r2)), cross = glm::cross(r1V, r2V).z;

	if (cross < 0)
		currPhase = (2 * glm::pi<double>()) - currPhase;

	double tt = (phase - currPhase) / (w2 - w1);

	synT = (2 * glm::pi<double>()) / (w1 - w2);

	if (tt < 0) tt += synT;

	return dt + tt;
}

double System::bodyDistTo(int b1, int b2, double dt) {
	glm::dvec3 r1 = bodies[b1]->getPV(dt, false).Pos;
	glm::dvec3 r2 = bodies[b2]->getPV(dt, false).Pos;

	if (b1 == 0)
		return  glm::length(r2);
	if (b2 == 0)
		return  glm::length(r1);

	return glm::length(r1 - r2);
}

// convert WWW MMM DD HH:MM:SS YYYY to YYYY MM DD
char* chrono2SpiceStr(const char* input) {
	const char* month_src = input + 4;
	const char* day_src = input + 8;
	const char* year_src = input + 20;

	int day, year;
	char month[4] = {};
	std::sscanf(month_src, "%3s", month);
	std::sscanf(day_src, "%d", &day);
	std::sscanf(year_src, "%d", &year);

	static char result[16] = {};
	std::snprintf(result, sizeof(result), "%04d %s %02d", year, month, day);
	return result;
}

System::~System() {
	for (auto& body : bodies) {
		body = nullptr;
	}
	for (auto& body : lightBodies) {
		body = nullptr;
	}
	for (auto& body : dullBodies) {
		body = nullptr;
	}
	for (auto& pos : bodyPos) {
		pos = nullptr;
	}
	for (auto& pos : satPos) {
		pos = nullptr;
	}
	for (auto& body : bodiesActual) {
		body.~Mesh();
	}
	dS.Delete();
	lS.Delete();
	dullShader = nullptr;
	lightShader = nullptr;
	if (persistent_memory_address != nullptr) {
		free((void*)persistent_memory_address);
		persistent_memory_address = nullptr;
	}

}


// time functions


void System::SystemTime() {
	char mess[30];
	struct time_block currTime =  { std::chrono::system_clock::now(), *mess, 0};
	currTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds>
		(currTime.timeUnit).time_since_epoch().count();
	auto hold = std::chrono::system_clock::to_time_t(currTime.timeUnit);
	ctime_s(currTime.timeString, sizeof(currTime.timeString), &hold );
	sysTime = currTime;
	simTime = currTime;
}

void System::time_block_ms_add(time_block &someTime, int sum, bool sign) {
	if (sum > 1000) {
		// throw exception for non three digit sum
	}
	int corr = 0;
	sign ? corr = 1: corr = -1;
	someTime.ms += sum;
	std::string num = std::to_string(someTime.ms);
	// handles overflowing ms
	if (num.length() > 3) {
		int overAmt = stoi(num.substr(0, num.length() - 3)) * 1000;
		someTime.ms -= overAmt;

		// adds reverse remainder to the total sum
		auto timeUnitT = std::chrono::system_clock::to_time_t((someTime.timeUnit));
		timeUnitT += corr * overAmt / 1000;
		someTime.timeUnit = std::chrono::system_clock::from_time_t((timeUnitT));
		ctime_s(someTime.timeString, sizeof(someTime.timeString), &timeUnitT);
		someTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds>
			(someTime.timeUnit).time_since_epoch().count();
	}
}

void System::WarpClockSet(const int currWarp) {
	// find current time and time elapsed since last update

	char mess[30];
	struct time_block currTime = { std::chrono::system_clock::now(), *mess, 0 };
	currTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds>
		(currTime.timeUnit).time_since_epoch().count();

	std::chrono::duration<double>diff = currTime.timeUnit - sysTime.timeUnit;

	// sample time allows for time updates roughly equivalent to Mesh::Orbit calls. Decrease if bodies jitter under time warp+
	float diffThreshold = 1 / (float)50;
	if (diff.count() >= diffThreshold) {
		bool sign;
		currWarp > 0 ? sign = true : sign = false;
		double diffWarp = diff.count() * (double)currWarp;
		if (currWarp < (1/diffThreshold)) { 
			//update ms timing
			time_block_ms_add(simTime, (int)abs((diffWarp*1000)), sign);
		}
		// update with time warp multiplier
		simTime.timeUnit += std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(diffWarp));
		auto pastTimeUnit = std::chrono::system_clock::to_time_t((simTime.timeUnit));

		// update sim time and reset sys time
		ctime_s(simTime.timeString, sizeof(simTime.timeString), &pastTimeUnit);
		simTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds> // simTime millisecond additions aren't saved?
			(simTime.timeUnit).time_since_epoch().count();

		sysTime = currTime;
	}
}

void System::ArgClockSet(double dt) {
	time_t sec = static_cast<time_t>(dt);
	simTime.timeUnit = std::chrono::system_clock::from_time_t(sec);

	ctime_s(simTime.timeString, sizeof(simTime.timeString), &sec);
	simTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds>
		(simTime.timeUnit).time_since_epoch().count();

}

void stateVecToSim(char* vec, state& someState) {
	// read line of ephemeris data and transform into state info in sim frame
	char* endPtr, dVec[300];
	strncpy(dVec, vec, strlen(vec));

	char* token = strtok(dVec, ",");
	double JDTDB = atof(token);
	someState.time = (JDTDB - 2440587.5) * 86400.0; // to UNIX
	
	token = strtok(NULL, ",");
	char dummy[30];
	strncpy(dummy, token + 5, 18);
	dummy[18] = '\0';
	someState.date = dummy; // can compare directly to maneuver data

	token = strtok(NULL, ",");
	someState.pos.x = strtod(token, &endPtr);

	token = strtok(NULL, ",");
	someState.pos.y = strtod(token, &endPtr);

	token = strtok(NULL, ",");
	someState.pos.z = strtod(token, &endPtr);

	token = strtok(NULL, ",");
	someState.vel.x = strtod(token, &endPtr);

	token = strtok(NULL, ",");
	someState.vel.y = strtod(token, &endPtr);

	token = strtok(NULL, ","); // atof doesn't seem to work for when E is to a negative integer, maybe the double conversion
	someState.vel.z = strtod(token, &endPtr);
}

// below is for adding / removing from persistent memory file
void sat2CSV(ArtSat& sat, char* buff, int &buffSize) {
	char temp[10000] = "";
	strncat(temp, sat.name, strlen(sat.name));
	int pl = strlen(temp);
	temp[pl++] = ',';
	for (int i = 0; i < sat.maneuvers.size(); i++) {
		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].time);
		pl = strlen(temp);
		
		strncat(temp, sat.maneuvers[i].name, strlen(sat.maneuvers[i].name));
		pl = strlen(temp);
		temp[pl++] = ',';

		strncat(temp, sat.maneuvers[i].desc, strlen(sat.maneuvers[i].desc));
		pl = strlen(temp);
		temp[pl++] = ',';

		// soi
		snprintf(temp + pl, sizeof(temp) - pl, "%d,", sat.maneuvers[i].soi);
		pl = strlen(temp);

		// old State
		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].origState.Pos.x);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].origState.Pos.y);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].origState.Pos.z);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].origState.Vel.x);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].origState.Vel.y);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].origState.Vel.z);
		pl = strlen(temp);		

		// new State
		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].newState.Pos.x);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].newState.Pos.y);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].newState.Pos.z);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].newState.Vel.x);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].newState.Vel.y);
		pl = strlen(temp);

		snprintf(temp + pl, sizeof(temp) - pl, "%9.9f,", sat.maneuvers[i].newState.Vel.z);
		pl = strlen(temp);
	}
	temp[strlen(temp) + 1] = '\0';
	strncpy(buff, temp, strlen(temp));
	buffSize = strlen(temp);
}

void csv2Sat(ArtSat& sat, char* buff, std::vector<Mesh*> *bodies) {
	char* token = strtok(buff, ",");
	sat.name = _strdup(token);

	char* endptr;
	int soi;
	double time;
	pvUnit oState, nState;

	token = strtok(NULL, ",");
	while (token != NULL) {
		time = strtod(token, &endptr);

		token = strtok(NULL, ",");
		char* name = new char[strlen(token) + 1];
		strncpy(name, token, strlen(token) + 1);

		token = strtok(NULL, ",");
		char* desc = new char[strlen(token) + 1];
		strncpy(desc, token, strlen(token) + 1);
		
		// soi
		token = strtok(NULL, ",");
		soi = strtod(token, &endptr);

		// old State
		token = strtok(NULL, ",");
		oState.Pos.x = strtod(token, &endptr);

		token = strtok(NULL, ",");
		oState.Pos.y = strtod(token, &endptr);

		token = strtok(NULL, ",");
		oState.Pos.z = strtod(token, &endptr);

		token = strtok(NULL, ",");
		oState.Vel.x = strtod(token, &endptr);

		token = strtok(NULL, ",");
		oState.Vel.y = strtod(token, &endptr);

		token = strtok(NULL, ",");
		oState.Vel.z = strtod(token, &endptr);

		// new State
		token = strtok(NULL, ",");
		nState.Pos.x = strtod(token, &endptr);

		token = strtok(NULL, ",");
		nState.Pos.y = strtod(token, &endptr);

		token = strtok(NULL, ",");
		nState.Pos.z = strtod(token, &endptr);

		token = strtok(NULL, ",");
		nState.Vel.x = strtod(token, &endptr);

		token = strtok(NULL, ",");
		nState.Vel.y = strtod(token, &endptr);

		token = strtok(NULL, ",");
		nState.Vel.z = strtod(token, &endptr);

		if (sat.maneuvers.size() == 0 && bodies != nullptr) 
			sat.ArtSatPlan(oState, time, soi, *bodies);
		else
			sat.maneuvers.push_back({ oState, nState, time, name, desc, soi });
		
		token = strtok(NULL, ",");
	}
	sat.lastEphTime = sat.maneuvers[sat.maneuvers.size() - 1].time;
}


