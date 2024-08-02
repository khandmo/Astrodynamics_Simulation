#include "System.h"


System::System() {
	// initialize all solar system bodies, a body's gravitational source must be initialized before that body
	// initialize radius as true radius in km divided by a factor of 6100/4550000000 to be consistent with orbital distances
	bodiesActual.push_back(initBody("sun", "Textures/SQmercury.jpg", glm::vec3(0.0f, 0.0f, 0.0f), 0.9335547255f, 0.0f, 0.0f, 0.0f, true, false, -1, 10, 10)); // CODE FIX FOR soiID if it is -1 to orbit the center of the universe or stay still
	bodiesActual.push_back(initBody("mercury", "Textures/SQmercury.jpg", glm::vec3(61.4f, 0.0f, 0.0f), 0.0032708066f, 0.0f, 2, 0.0600068844f, false, false, 0, 1, 199));
	bodiesActual.push_back(initBody("venus", "Textures/SQvenus.jpg", glm::vec3(143.6f, 0.0f, 0.0f), 0.0081134022f, 0.0f, 3, 0.0434617764f, false, false, 0, 2, 299));
	bodiesActual.push_back(initBody("earth", "Textures/earth4096.jpg", glm::vec3(200.0f, 0.0f, 0.0f), 0.0085413407f, 0.0f, 23.5, 10.56121166f, false, false, 0, 3, 399));
	bodiesActual.push_back(initBody("moon", "Textures/moon4096.jpg", glm::vec3(205.229f, 0.0f, 0.0f), 0.00232926115f, 0.0f, 1.5, 0.35800717f, false, false, 3, 301, 301));
	bodiesActual.push_back(initBody("mars", "Textures/SQmars.jpg", glm::vec3(276.28f, 0.0f, 0.0f), 0.0045441648f, 0.0f, 25, 10.57f, false, false, 0, 4, 499));
	bodiesActual.push_back(initBody("jupiter", "Textures/SQjupiter.jpg", glm::vec3(990.0f, 0.0f, 0.0f), 0.0937268352f, 0.0f, 3, 25.64f, false, false, 0, 5, 599));
	bodiesActual.push_back(initBody("saturn", "Textures/SQsaturn.jpg", glm::vec3(1808.0f, 0.0f, 0.0f), 0.0780692747f, 0.0f, 26.73, 23.6886f, false, false, 0, 6, 699));
	bodiesActual.push_back(initBody("saturnRings", "Textures/SQsaturnRings.jpg", glm::vec3(1808.0f, 0.0f, 0.0f), 0.1005494506f, 0.1876923077f, 26.73, 21.0f, false, true, 0, 6, 699)); 
	bodiesActual.push_back(initBody("uranus", "Textures/SQuranus.jpg", glm::vec3(3680.0f, 0.0f, 0.0f), 0.0340018022f, 0.0f, 97.7, 14.6939f, false, false, 0, 7, 799));
	bodiesActual.push_back(initBody("neptune", "Textures/SQneptune.jpg", glm::vec3(5962.0f, 0.0f, 0.0f), 0.0330097143f, 0.0f, 28, 15.8418f, false, false, 0, 8, 899));
	
	// transplant bodies addresses
	for (int i = 0; i < bodiesActual.size(); i++) {
		bodies.push_back(&bodiesActual[i]);
		if (bodies[i]->isLightSource) {
			lightBodies.push_back(bodies[i]);
		}
		else {
			dullBodies.push_back(bodies[i]);
		}
		if (bodies[i]->soiID != -1) {
			bodies[i]->gravSource = &bodiesActual[bodies[i]->soiID];
		}
	}

	// time memory save
	sysTime = *(time_block*) malloc(sizeof(time_block));
	simTime = *(time_block*) malloc(sizeof(time_block));

	// set shaders
	shaderSet();
	updateBodyState(); // shaders stable here
}

Mesh System::initBody(const char* name, const char* texFilePath, glm::vec3 pos, float radius, float outerRadius, float axialTilt, float angleOfRot,  bool isLight, bool areRings, int soiID, int baryID, int spiceID) {
	
	// init texture
	Texture tex[] = { Texture(texFilePath, "diffuse", 0, GL_RGB, GL_UNSIGNED_BYTE) };

	// init object
	Object obj;
	if (!areRings) {
		obj.Sphere(radius, 0.2f, 0.4f, 0.3f); // initialize as sphere, 3 other floats don't matter
	}
	else {
		obj.Rings(radius, outerRadius, 0.2f, 0.4f, 0.3f);
	}
	std::vector <Texture> objTex(tex, tex + sizeof(tex) / sizeof(Texture));
	glm::vec4 lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f); // default color, dummy for dull bodies

	Shader shader = dS;
	if (isLight) {
		shader = lS;
	}

	SystemTime();
	Mesh body(name, obj.vertices, obj.indices, objTex, isLight, areRings, lightColor, pos, &shader, baryID, spiceID, sysTime.time_in_sec); // velocity will be updated with SPICE integration

	// Set Properties
	body.soiID = soiID;
	body.AxialTilt(axialTilt);
	body.radRot = angleOfRot; // assign rotation speed
	bodyRadii.push_back(radius);

	return body;
}

void System::updateBodyState() { // System holds it's own positions, gets updated from Mesh --- System triggers Move and calls for velocity from body
	// update vector of bodyPos in glm vec3 format
	for (int i = 0; i < bodies.size(); i++) {
		//std::cout << bodies[i]->name << " update body state shader id " << bodies[i]->ShaderProgram->ID << '\n';
		if (bodyPos.size() >= i + 1)
			bodyPos[i] = &(bodies[i]->Pos);
		else
			bodyPos.push_back(&(bodies[i]->Pos));
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
	someTime.ms += sum; // addition isn't being saved
	std::string num = std::to_string(someTime.ms);
	// handles overflowing ms
	if (num.length() > 3) {
		int overAmt = stoi(num.substr(0, num.length() - 3)) * 1000;
		someTime.ms -= overAmt;

		// adds reverse remainder to the total sum
		auto timeUnitT = std::chrono::system_clock::to_time_t((someTime.timeUnit));
		sign == true ? timeUnitT += overAmt/1000 : timeUnitT -= overAmt/1000;
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
		pastTimeUnit += diffWarp;
		simTime.timeUnit = std::chrono::system_clock::from_time_t(pastTimeUnit);

		// update sim time and reset sys time
		ctime_s(simTime.timeString, sizeof(simTime.timeString), &pastTimeUnit);
		simTime.time_in_sec = std::chrono::time_point_cast<std::chrono::seconds>
			(simTime.timeUnit).time_since_epoch().count();

		sysTime = currTime;
		std::cout << simTime.timeString << '\n';
	}
}




void System::deleteSystem() {

}
