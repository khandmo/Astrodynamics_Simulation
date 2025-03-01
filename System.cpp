#include "System.h"
#include <cstring>

//double distanceFind(glm::vec3 state1, glm::vec3 state2);
void stateVecToSim(char* vec, state& someState);

System::System() {
	// initialize all solar system bodies, a body's gravitational source must be initialized before that body
	// initialize radius as true radius in km divided by a factor of 6100/4550000000 to be consistent with orbital distances
	bodiesActual.push_back(initBody("Sun", "Textures/SQmercury.jpg", 1.989f * pow(10, 10), 696340.0f, 0.0f, 0.0f, 0.0f, true, false, "0", 10, 10, 0)); // CODE FIX FOR soiID if it is -1 to orbit the center of the universe or stay still
	bodiesActual.push_back(initBody("Mercury", "Textures/SQmercury.jpg", 3.3011f * pow(10, 3), 2439.7f, 0.0f, 2, 0.0600068844f, false, false, "0", 1, 199, 88));
	bodiesActual.push_back(initBody("Venus", "Textures/SQvenus.jpg", 4.8675f * pow(10, 4), 6051.8f, 0.0f, 3, 0.0434617764f, false, false, "0", 2, 299, 225));
	bodiesActual.push_back(initBody("Earth", "Textures/earth4096.jpg", 5.97237f * pow(10, 4), 6371.0f, 0.0f, 23.5, 10.56121166f, false, false, "0", 3, 399, 365));
	bodiesActual.push_back(initBody("Moon", "Textures/moon4096.jpg", 7.342f * pow(10, 2), 1737.4f, 0.0f, 1.5, 0.35800717f, false, false, "3", 301, 301, 27));
	bodiesActual.push_back(initBody("Mars", "Textures/SQmars.jpg", 6.4171f * pow(10, 3), 3389.5f, 0.0f, 25, 10.57f, false, false, "0", 4, 499, 687));
	//bodiesActual.push_back(initBody("Jupiter", "Textures/SQjupiter.jpg", 1.8982f * pow(10, 7), 69911.0f, 0.0f, 3, 25.64f, false, false, "0", 5, 599, 4331));
	//bodiesActual.push_back(initBody("Saturn", "Textures/SQsaturn.jpg", 5.6834f * pow(10, 6), 58232.0f, 0.0f, 26.73, 23.6886f, false, false, "0", 6, 699, 10759));
	//bodiesActual.push_back(initBody("saturnRings", "Textures/SQsaturnRings.jpg", 0.0f, 75000.0f, 140000.0f, 26.73, 21.0f, false, true, "0", 6, 699, 10759));
	//bodiesActual.push_back(initBody("Uranus", "Textures/SQuranus.jpg", 8.681 * pow(10, 5), 25362.0f, 0.0f, 97.7, 14.6939f, false, false, "0", 7, 799, 30689));
	//bodiesActual.push_back(initBody("Neptune", "Textures/SQneptune.jpg", 1.02413 * pow(10, 6), 24622.0f, 0.0f, 28, 15.8418f, false, false, "0", 8, 899, 60182));
	
	// transplant bodies addresses
	for (int i = 0; i < bodiesActual.size(); i++) {
		bodies.push_back(&bodiesActual[i]);
		if (bodies[i]->isLightSource) {
			lightBodies.push_back(bodies[i]);
		}
		else {
			dullBodies.push_back(bodies[i]);
		}
		if (bodies[i]->soiID != "-1"){
			bodies[i]->gravSource = &bodiesActual[std::stoi(bodies[i]->soiID)];
		}
	}

	// init art sats
	// program to take artSats and name of ephemeris data, process and add mission + maneuver to persistent memory

	initSat("Artemis 1", "horizons_results_raw.txt", artSats);

	// time memory save
	sysTime = *(time_block*) malloc(sizeof(time_block));
	simTime = *(time_block*) malloc(sizeof(time_block));

	// set shaders
	shaderSet();
	updateBodyState(); // shaders stable here
}

Mesh System::initBody(const char* name, const char* texFilePath, float mass, float radius, float outerRadius, float axialTilt, float angleOfRot, bool isLight, bool areRings, const char* soiID, int baryID, int spiceID, int orbPeriod) {
	// init texture
	Texture tex[] = { Texture(texFilePath, "diffuse", 0, GL_RGB, GL_UNSIGNED_BYTE) };

	// init object
	Object obj;
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

	SystemTime();
	Mesh body(name, obj.vertices, obj.indices, objTex, radius, mass, isLight, areRings, &shader, soiID, baryID, spiceID, sysTime.time_in_sec, orbPeriod); // velocity will be updated with SPICE integration
	// Set Properties	
	body.AxialTilt(axialTilt);
	body.radRot = angleOfRot; // assign rotation speed
	bodyRadii.push_back(radius * LENGTH_SCALE);

	return body;
}

void System::initSat(const char* name, const char* eph, std::vector<ArtSat>& artSats) {
	// hold maneuvers
	char* mane[100];

	// parse text file
	std::fstream dataStream(eph, std::fstream::in); // can read and write
	if (!dataStream) {
		std::cout << "File failed to open / create\n";
		dataStream.clear();
		return;
	}

	bool close = false;
	int maneIdx = 0;
	while (!close) {
		// get maneuver information
		char* buff = new char[300];
		dataStream.getline(buff, 300);

		if (buff[0] == buff[1] && buff[1] == buff[2] && buff[2] == '*' || maneIdx > 19) { // *** terminates maneuver information
			break;
		}
		mane[maneIdx++] = buff;
	}

	char buff[300];
	state someState;
	dataStream.getline(buff, 300);
	stateVecToSim(buff, someState);

	ArtSat* test = new ArtSat();
	double testTime = someState.time;
	test->ArtSatPlan({ someState.pos, someState.vel }, testTime, 3, bodies);
	test->name = name;

	// initialize at first line time, state
	// add maneuvers when the time equals the next mane
	// just modify velocity
	// need little function for parsing the maneuver and persistent data to apply logic to

	// need to know if mission is over, when, and if not what the current state is (cheat) and 
	// update in a file with the ability to read and propogate from the last input location

	// grab maneuvers, end state
	char altBuff[300];
	int maneIdx2 = 1;
	while (1) {
		strcpy(altBuff, buff);
		dataStream.getline(buff, 300);

		if (buff[0] == buff[1] && buff[1] == buff[2] && buff[2] == '*') { // *** terminates maneuver information
			break;
		}
		/* maneuver importing below
		char* altBuff2 = new char, * altBuffTime = new char;
		strncpy(altBuff2, buff, sizeof(buff));
		altBuff2 += 24;
		strncpy(altBuffTime, altBuff2, sizeof(buff) - 24);
		altBuffTime += 12;
		altBuff2[12] = '\0';
		altBuffTime[2] = '\0';

		char* subS = strstr(mane[maneIdx2], altBuff2);
		char* subsubS = subS + 12;
		subsubS[2] = '\0';
		if (subS != nullptr) { // if date of a maneuver

			if (std::stoi(altBuffTime) - std::stoi(subsubS) < 6){// ****** assume ephermis data taken 6 hours apart
				// if time right after a maneuver
				// add maneuver (in buff) to satellite, 

				maneIdx2++;
			}
			else if (std::stoi(altBuffTime) > 18) {
				// if first thing tomorrow is soonest after maneuver

			}

		}

		delete altBuff2;
		delete altBuffTime;
		*/
	}
	char* token = strtok(altBuff, ";");
	double endTime = atof(token);
	endTime = (endTime - 2440587.5) * 86400.0; // to UNIX
	// add to artSat, check time at update

	dataStream.close();


	test->lastEphTime = endTime;
	artSats.push_back(*test);


	// free all mane's
	while (maneIdx != 0) {
		delete mane[--maneIdx];
	}

}

void System::ArtSatHandle(std::vector<Mesh*> bodies, Camera* camera, double dt, int tW) {

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
		//std::cout << simTime.timeString << '\n';
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
	char* endPtr;

	char* token = strtok(vec, ",");
	double JDTDB = atof(token);
	someState.time = (JDTDB - 2440587.5) * 86400.0; // to UNIX
	
	token = strtok(NULL, ",");
	char dummy[30];
	strncpy(dummy, token + 5, 18);
	someState.date = dummy + '\0'; // can compare directly to maneuver data

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



void System::deleteSystem() {

	// free all art sats


}

