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
	erract_c("SET", 0, (SpiceChar*)"RETURN");

	SystemTime();

	// initialize all solar system bodies, a body's gravitational source must be initialized before that body
	// initialize radius as true radius in km divided by a factor of 6100/4550000000 to be consistent with orbital distances
	bodiesActual.push_back(initBody("Sun", "Textures/SQmercury.jpg", 1.989f * pow(10, 10), 696340.0f, 0.0f, 0.0f, 0.0f, true, false, "10", "0", 10, 10, 0)); // CODE FIX FOR soiID if it is -1 to orbit the center of the universe or stay still
	bodiesActual.push_back(initBody("Mercury", "Textures/SQmercury.jpg", 3.3011f * pow(10, 3), 2439.7f, 0.0f, 2, 0.0600068844f, false, false, "10", "0", 1, 199, 88));
	bodiesActual.push_back(initBody("Venus", "Textures/SQvenus.jpg", 4.8675f * pow(10, 4), 6051.8f, 0.0f, 3, 0.0434617764f, false, false, "10", "0", 2, 299, 225));
	bodiesActual.push_back(initBody("Earth", "Textures/earth4096.jpg", 5.97237f * pow(10, 4), 6371.0f, 0.0f, 23.5, 10.56121166f, false, false, "10", "0", 3, 399, 365));
	bodiesActual.push_back(initBody("Moon", "Textures/moon4096.jpg", 7.342f * pow(10, 2), 1737.4f, 0.0f, 1.5, 0.35800717f, false, false, "399", "3", 301, 301, 27));
	bodiesActual.push_back(initBody("Mars", "Textures/SQmars.jpg", 6.4171f * pow(10, 3), 3389.5f, 0.0f, 25, 10.57f, false, false, "10", "0", 4, 499, 687));
	bodiesActual.push_back(initBody("Jupiter", "Textures/SQjupiter.jpg", 1.8982f * pow(10, 7), 69911.0f, 0.0f, 3, 25.64f, false, false, "10", "0", 5, 599, 4331));
	bodiesActual.push_back(initBody("Saturn", "Textures/SQsaturn.jpg", 5.6834f * pow(10, 6), 58232.0f, 0.0f, 26.73, 23.6886f, false, false, "10", "0", 6, 699, 10759));
	//bodiesActual.push_back(initBody("saturnRings", "Textures/SQsaturnRings.jpg", 0.0f, 75000.0f, 140000.0f, 26.73, 21.0f, false, true, "10", "0", 6, 699, 10759));
	bodiesActual.push_back(initBody("Uranus", "Textures/SQuranus.jpg", 8.681 * pow(10, 5), 25362.0f, 0.0f, 97.7, 14.6939f, false, false, "10", "0", 7, 799, 30689));
	bodiesActual.push_back(initBody("Neptune", "Textures/SQneptune.jpg", 1.02413 * pow(10, 6), 24622.0f, 0.0f, 28, 15.8418f, false, false, "10", "0", 8, 899, 60182));
	
	// transplant bodies addresses
	for (int i = 0; i < bodiesActual.size(); i++) {
		bodies.push_back(&bodiesActual[i]);
		bodies[i]->spiceMtx = &mtx;
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
	//initPersistSats("persistent_sats.txt");
	//initSat("Artemis 1", "horizons_results_raw.txt", "persistent_sats.txt");

	// time memory save
	sysTime = *(time_block*) malloc(sizeof(time_block));
	simTime = *(time_block*) malloc(sizeof(time_block));

	// set shaders
	shaderSet();
	updateBodyState(); // shaders stable here
}

Mesh System::initBody(const char* name, const char* texFilePath, float mass, float radius, float outerRadius, float axialTilt, float angleOfRot, bool isLight, bool areRings, const char* soiID, const char* soiIdx, int baryID, int spiceID, int orbPeriod) {
	// init texture
	Texture tex[] = { Texture(texFilePath, "diffuse", 0, GL_RGB, GL_UNSIGNED_BYTE) };

	// init object
	Object obj; // ******** 891.33 KB heap
	if (!areRings) {
		obj.Sphere(radius * LENGTH_SCALE, 0.2f, 0.4f, 0.3f); // initialize as sphere, 3 other floats don't matter
	}
	else {
		obj.Rings(radius * LENGTH_SCALE, outerRadius * LENGTH_SCALE, 0.2f, 0.4f, 0.3f);
	}
	std::vector <Texture> objTex(tex, tex + sizeof(tex) / sizeof(Texture));

	Shader shader = dS;
	if (isLight) {
		shader = lS;
	}

	// masses are all / 10^20 for the sake of transportation

	 // ********** mesh init has 1,806 KB heap
	Mesh body(name, obj.vertices, obj.indices, objTex, radius, mass, isLight, areRings, &shader, soiID, soiIdx, baryID, spiceID, sysTime.time_in_sec, orbPeriod); // velocity will be updated with SPICE integration
	// Set Properties	
	body.AxialTilt(axialTilt);
	body.radRot = angleOfRot; // assign rotation speed
	bodyRadii.push_back(radius * LENGTH_SCALE);

	return body;
}

void System::initSat(const char* name, const char* eph, const char* prstnt) {

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
	artSats.push_back(*sat);


	// free all held maneuver strings
	while (maneIdx != 0) {
		delete mane[--maneIdx];
	}

	std::cout << "new satellite " << sat->name << " added\n";

	// add sat to end of persistent txt (w/ new line at end)
	std::fstream dataStream2(prstnt, std::fstream::out | std::fstream::app); // should just read
	if (!dataStream2) {
		std::cout << "File " << prstnt << " failed to open / create\n";
		dataStream.clear();
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

}

void System::initPersistSats(const char* file) {

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
		csv2Sat(*sat, buff, &bodies);
		artSats.push_back(*sat);
		dataStream.getline(buff, 10000);
	}


	dataStream.clear();
	dataStream.close();

	// sat info is wiped after this function exits
}

void System::ArtSatHandle(Camera* camera, double dt, int tW) {
	// if time warp over certain amount, run this in thread
	// push estimate positions to screen until time warp is brought back down and computations can complete 
	// use nodes for estimates, refresh traj if all out of nodes
	// can assume fit local nodes to low poly estimates to estimate position for visual purposes


	for (int i = 0; i < artSats.size(); i++) {
		artSats[i].ArtSatUpdState(bodies, dt, tW,  0);
		artSats[i].ArtSatRender(camera, *(bodies[0]));
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
	}
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

	float planetDistLowBound = 3, planetDistHighBound = 15, planetDistLowLowBound = 1.4f;
	float moonDistLowBound = 0.08f, moonDistHighBound = 0.4f; // *********** should use radius for these to scale, should make opacity calc helper fxn
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

	glm::dvec3 r1V = bodies[b1]->getPV(dt, false, false).Pos;
	glm::dvec3 r2V = bodies[b2]->getPV(dt, false, false).Pos;

	r1 = glm::length(r1V);
	r2 = glm::length(r2V);

	tTime = glm::pi<double>() * sqrt(pow((r1 + r2) / 2, 3) / mu);

	w1 = sqrt(mu / pow(r1, 3)); w2 = sqrt(mu / pow(r2, 3));

	phase = glm::pi<double>() - ((w1 - w2) * tTime);

	// compute current phase
	double currPhase = acos(glm::dot(r1V, r2V) / (r1 * r2)), cross = glm::cross(r1V, r2V).z;

	if (cross < 0)
		currPhase = (2 * glm::pi<double>()) - currPhase;

	double tt = (phase - currPhase) / (w2 - w1);

	synT = (2 * glm::pi<double>()) / (w1 - w2);

	return dt + tt;
}

double System::bodyDistTo(int b1, int b2, double dt) {
	glm::dvec3 r1 = bodies[b1]->getPV(dt, false, false).Pos;
	glm::dvec3 r2 = bodies[b2]->getPV(dt, false, false).Pos;

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

	std::chrono::duration<double> diff = 
		std::chrono::time_point_cast<std::chrono::milliseconds>(currTime.timeUnit)
		- std::chrono::time_point_cast<std::chrono::milliseconds>(sysTime.timeUnit);

	// sample time allows for time updates roughly equivalent to Mesh::Orbit calls. Decrease if bodies jitter under time warp+
	float diffThreshold = 1 / 150;
	if (diff.count() >= diffThreshold) {
		bool sign;
		currWarp > 0 ? sign = true : sign = false;
		double diffWarp = diff.count() * currWarp;
		if (currWarp < 1/diffThreshold) { 
			//update ms timing
			time_block_ms_add(simTime, (int)abs((diffWarp*1000)), sign);
		}
		// update with time warp multiplier
		auto pastTimeUnit = std::chrono::system_clock::to_time_t((simTime.timeUnit));
		pastTimeUnit += static_cast<time_t>(diffWarp);
		simTime.timeUnit = std::chrono::system_clock::from_time_t(pastTimeUnit);

		// update sim time and reset sys time
		ctime_s(simTime.timeString, sizeof(simTime.timeString), &pastTimeUnit);
		simTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds>
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
			sat.ArtSatPlan(oState, time, 3, *bodies);
		else
			sat.maneuvers.push_back({ oState, nState, time, name, desc });
		
		token = strtok(NULL, ",");
	}
	sat.lastEphTime = sat.maneuvers[sat.maneuvers.size() - 1].time;
}


