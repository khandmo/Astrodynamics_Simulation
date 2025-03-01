#include "ArtSat.h"

pvUnit RK4(pvUnit pv, double M, double dt);
pvUnit leapfrog(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep);
glm::dvec3 onePN(pvUnit pv, double M);
glm::dvec3 accel_comp(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt);
double vecMag(glm::dvec3 vec);
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiID, double dt, float timeStep, double &E0);

std::vector <double> createGaussianKernel(int size, int stdDev);
int gSmooth(std::vector<double> *kernel, std::vector<glm::dvec3> *noisy, std::vector<glm::dvec3> *smooth);
float getAngle(glm::vec3 v1, glm::vec3 v2);
void simCartToSph(glm::dvec3& cart);

double totalEnergy(pvUnit pv, double M, double* KE, double* PE);

ArtSat::ArtSat() {

	// init shader device, will need to get uniform for camera info to render
	ArtSat::pathDevice = geom_shdr_lines_init_device();
	// init box 
	sat = new Object;
	sat->Box(.0005, 0, 1, 0.2);

	apoapsis = new poi;
	periapsis = new poi;
	stat = new stats;

	state = new pvUnit;
	stateButChanged = new pvUnit;

	
	stateTime = 0;

	VAOu.Bind();
	VBO VBO(sat->vertices);
	EBO EBO(sat->indices);

	VAOu.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
	VAOu.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
	VAOu.LinkAttrib(VBO, 2, 3, GL_FLOAT, sizeof(Vertex), (void*)(6 * sizeof(float)));
	VAOu.LinkAttrib(VBO, 3, 2, GL_FLOAT, sizeof(Vertex), (void*)(9 * sizeof(float)));
	VAOu.Unbind();
	VBO.Unbind();
	EBO.Unbind();
}

// do not pass soiID, needs index of soi in bodies
int ArtSat::ArtSatPlan(pvUnit pv, double dt, int soiIndex, std::vector <Mesh*> bodies) {
	// on the fly position and velocity modification initialization with visual updates

	// only runs program if lineBuff if a new input is given
	if (prevPV == nullptr || *prevPV != pv) { // no memory held here sometimes? (*prevPV)
		if (prevPV == nullptr)
			prevPV = new pvUnit(pv);
		else {
			*prevPV = pv;
		}

		soiIdx = soiIndex;

		pvUnit* pvSC = new pvUnit(pv);
		stateChange(&(pvSC->Pos), &(pvSC->Vel));


		// add pos as poi to carry
		*ArtSat::state = pv;
		*ArtSat::stateButChanged = *pvSC;
		ArtSat::stateTime = dt;

		stat->initTime = dt;

		maneuvers.push_back({ pv, pv, dt, "launch", "orbital insertion"});

		// chart trajectory
		chartTraj(pv, bodies, dt);
		chartApproach(bodies, 4);
	}
	return 1;
}



// should perform this on a copy of the probe first and when save is triggered splice into System save
void ArtSat::ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, double dt, const char name[30], const char desc[30]) {
	// modify velocity (instantaneously or over some dt burn time)

	std::chrono::time_point<std::chrono::system_clock> t1, t2, t3, t4, t5;
	t1 = std::chrono::system_clock::now();
	// probe forward to find sat pv @ dt
	int timeTill = std::max(dt - stateTime, (double)1);
	int acc = 0; double dummy = 0;
	while (acc != timeTill) { // would love to set up a thread to do this in the back and provide a 30 update in the meantime
		updatePV(state, bodies, soiIdx, dt + ++acc, 1, dummy);
	}
	t2 = std::chrono::system_clock::now();
	pvUnit origState = *state;

	simCartToSph(state->Vel);
	state->Vel += deltaV; // add spherically
	state->Vel = sphToCart(state->Vel);
	state->Vel = { -state->Vel.x, state->Vel.z, state->Vel.y };

	maneuvers.push_back({origState, *state, dt, name, desc });
	t3 = std::chrono::system_clock::now();
	chartTraj(*state, bodies, dt);
	t4 = std::chrono::system_clock::now();
	chartApproach(bodies, 4);
	t5 = std::chrono::system_clock::now();

	std::chrono::duration<double> update, traj, appr;
	update = t2 - t1;
	traj = t4 - t3;
	appr = t5 - t4;

	int hook = 5;
}

void ArtSat::ArtSatUpdState(std::vector <Mesh*> bodies, double dt, int tW, double mod) {


	if (dt < maneuvers[0].time || lastEphTime != -1 && dt - lastEphTime > 60 * 60 * 24) { // mod is -1 at art sat creation
		inTime = false;
		return;
	}
	else {
		inTime = true;
	}

	// check if maneuver scheduled and execute
	if (mod != -1) {
		for (int i = 0; i < maneuvers.size(); i++) {
			if (maneuvers[i].time == dt) {

				if (tW < 15) {
					*state = maneuvers[i].origState;
				}
				else {
					*state = maneuvers[i].newState;
				}

				chartTraj(*state, bodies, dt);
				chartApproach(bodies, 4);
				return;
			}
		}
	}


	if (inTime) {

		// should refresh orbits at every turning point
		if (mk1 == nullptr) {
			chartApproach(bodies, 4);
		}
		else {
			mk1->simPos = mk1->fixedPos + *bodies[soiIdx]->Pos;
			mk2->simPos = mk2->fixedPos + *bodies[soiIdx]->Pos;
		}

		// handle state update
		int counter = dt - stateTime;
		double lastTime = stateTime, dummy = 0;
		while (counter > 0) {
			updatePV(state, bodies, soiIdx, lastTime++, 1.0f, dummy);
			counter--;
		}
		*stateButChanged = *state;
		stateChange(&(stateButChanged->Pos), &(stateButChanged->Vel));
		stateTime = dt;

		ArtSat::simPos = (glm::vec3)stateButChanged->Pos + *(bodies[soiIdx]->Pos);

		// handle stats update
		stat->timeToApo = abs(apoapsis->time - (dt - stat->initTime));
		stat->timeToPeri = abs(periapsis->time - (dt - stat->initTime));
		stat->MET = dt - stat->initTime; // need start time held too
		stat->distToSoi = vecMag(state->Pos) - bodies[soiIdx]->realRadius;

		// handle model
		glm::mat4 objModel = glm::mat4(1.0f);
		Model = glm::translate(objModel, simPos);

		// relativize path to soi position
		for (int i = 0; i < LINE_BUFF_SIZE_AS; i++) {
			relLB[i] = lineBuff[i];
			relLB[i].pos += *(bodies[soiIdx]->Pos);
			if (mod == 1) {
				relLB[i].col = { 0.0f, 0.3f, 1.0f, 0.43f };
			}
		}
	}
}


void ArtSat::ArtSatRender(Camera* camera, Mesh lightSource) {
	if (inTime) {
		// shader
		setShader(lightSource);
		// draw box
		sp.Activate();
		VAOu.Bind();

		camera->Matrix(sp, "camMatrix");
		glDrawElements(GL_TRIANGLES, sat->indices.size(), GL_UNSIGNED_INT, 0);

		// render lineBuff
		uniform_data_t uni;
		glm::mat4 mvp = camera->cameraMatrix * glm::mat4(1.0); // mult 4x4 glm - non zero, should check shader
		uni.mvp = &mvp[0][0];
		glm::vec4 vpt = glm::vec4(0, 0, (float)camera->width, (float)camera->height);
		uni.viewport = &vpt.z;
		glm::vec2 aa_radii = glm::vec2(2.0f, 2.0f);
		uni.aa_radius = &aa_radii.x;

		geom_shdr_lines_update(&pathDevice, &relLB,
			LINE_BUFF_SIZE_AS, sizeof(vertex_t), &uni);

		int buffSize = LINE_BUFF_SIZE_AS;
		if (lB_size_actual != -1)
			buffSize = lB_size_actual;
		geom_shdr_lines_render(&pathDevice, buffSize);

		mk1->MarkerRender(camera);
		mk2->MarkerRender(camera);
	}
}



ArtSat::~ArtSat() {
	// delete pvunit, lineBuff, obj Mesh, and locked pos and vel init
	delete apoapsis;
	delete periapsis;
	delete state;
	delete stateButChanged;
	delete prevPV;
	delete sat;
	delete stat;
	delete mk1;
	delete mk2;
}

void ArtSat::setShader(Mesh& lightSource) {
	sp.Activate();
	// model shader position uniform
	glUniformMatrix4fv(glGetUniformLocation(sp.ID, "model"), 1, GL_FALSE, glm::value_ptr(Model));

	// model shader color uniform (sun on earth) and shader uniform
	glUniform4f(glGetUniformLocation(sp.ID, "lightColor"), lightSource.Color.x, lightSource.Color.y, lightSource.Color.z, lightSource.Color.w);
	glUniform3f(glGetUniformLocation(sp.ID, "lightPos"), (lightSource.Pos)->x, (lightSource.Pos)->y, (lightSource.Pos)->z);

	// uniform for depth and default
	glUniformMatrix4fv(glGetUniformLocation(sp.ID, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lsMatrix));
	// uniform for default
	glUniform1i(glGetUniformLocation(sp.ID, "shadowMap"), 1);
}

void ArtSat::chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt) {
	// init vars
	double itTime = dt;
	float min = 60.0f;
	int flpr = -1;

	pvUnit* pvCurr = new pvUnit(pv);
	pvUnit* pvSC = new pvUnit(pv);

	stateChange(&(pvSC->Pos), &(pvSC->Vel));
	lineBuff[0] = vertex_t{ {pvSC->Pos, lineWidth}, lineColor };

	std::vector <glm::dvec3> dynLBN, dynLBS;
	std::vector <int> soi_list;

	bool close = false, dir = false, dir2 = false;
	double lastToPV, toPV = 0;
	int lastItTime = 0; double lastVelocity = 0, velocity = 0;
	double factor = 1;
	int tp = 0; // # of turning points
	int iterations = 0; // total iteration count
	int ratio = 20; // how to divide 1 minute into default time step (3 sec @ 20)
	poi apo{ {0,0,0}, 0 }, peri{ {0,0,0}, 0 }; // turning point init

	double* T = new double, * U = new double;
	double energy_0 = totalEnergy(*pvCurr, bodies[soiIdx]->mass, T, U);

	// init kernel
	std::vector<double> gKernel = createGaussianKernel(101, 15);


	std::chrono::time_point<std::chrono::system_clock> t1, t2, t3, t4, t5;
	
	

	t1 = std::chrono::system_clock::now();
	// scan step
	while (!close) {
		iterations++;
		// reset to forward time after sample pts collected for smoothing
		if (dynLBN.size() > ((gKernel.size() - 1) / 2) && flpr == -1) {
			itTime = dt;
			flpr = 1;
			*pvCurr = pv;
			glm::dvec3 dummy;
			for (int i = 0; i < (dynLBN.size() - 1) / 2; i++) {
				dummy = dynLBN[i];
				int dummier = dynLBN.size() - 1 - i;
				dynLBN[i] = dynLBN[dummier];
				dynLBN[dummier] = dummy;
			}
			t2 = std::chrono::system_clock::now();
		}

		// a & b check for turning points
		double a = abs(getAngle(pvCurr->Pos, pvCurr->Vel) - (glm::pi<double>() / 2));

		// generate new prediction
		updatePV(pvCurr, bodies, soiIdx, itTime, ((flpr * min) / ratio) * factor, energy_0);
		itTime += ((flpr * min) / ratio) * factor;

		double b = abs(getAngle(pvCurr->Pos, pvCurr->Vel) - (glm::pi<double>() / 2));
		double c = 0; // checks distance to soi

		// update time step based on velocity (could try acceleration)
		lastVelocity = velocity;
		velocity = vecMag(pvCurr->Vel);
		factor = exp(-((velocity - 9) * 3 / 5)); // could try lV vs. V - make factor large when deltaV is large and vice versa
		//factor = exp(-(abs(velocity - lastVelocity) * 2)); // fxn doesnt do the trick - need low diff to be a larger number (over 1), high diff to be closer
		// NEED a threshold like 9 to compare - would need the average difference over the orbit to be universal

		*pvSC = *pvCurr;
		stateChange(&(pvSC->Pos), &(pvSC->Vel));
		dynLBN.push_back(pvSC->Pos);

		// smoothing fxn
		gSmooth(&gKernel, &dynLBN, &dynLBS); // first pt of dynLBS is 0


		// general handling
		if (dynLBS.size() > 2) {
			soi_list.push_back((int)energy_0); // re-use variable, not flipping list after backtrack, vecMag(bodies[4]->getPV(dt, false).Pos - pvCurr->Pos)
			c = vecMag(dynLBS[dynLBS.size() - 1]);
			lastToPV = toPV;
			toPV = distanceFind(pv.Pos, dynLBS[dynLBS.size() - 1]);
			dir2 = dir;

			if (a >= b) // U growing
				dir = true;
			else
				dir = false;
		}

		// check for turning points / close trigger
		if (dynLBS.size() > 5) {
			if (dir != dir2 && (iterations - lastItTime) > 100) {
				tp++;
				lastItTime = iterations;

				if (apo.time == 0 || vecMag(apo.Pos) < vecMag(dynLBS[dynLBS.size() - 1])) {
					apo.Pos = dynLBS[dynLBS.size() - 1]; apo.time = itTime - dt;
				}

				if (peri.time == 0 || vecMag(peri.Pos) > vecMag(dynLBS[dynLBS.size() - 1])) {
					peri.Pos = dynLBS[dynLBS.size() - 1]; peri.time = itTime - dt;
				}
			}

			if (c < bodies[soiIdx]->radius || (tp == 4 && toPV > lastToPV) || iterations > 5000) // arbitrary constant
				close = true;
		}

	}
	stat->apoapsis = (vecMag(apo.Pos) / (LENGTH_SCALE)) - bodies[soiIdx]->realRadius;
	stat->periapsis = (vecMag(peri.Pos) / (LENGTH_SCALE)) - bodies[soiIdx]->realRadius;
	stat->orbitalPeriod = itTime - dt; // in seconds

	delete T; delete U;
	*apoapsis = apo;
	*periapsis = peri;



	t3 = std::chrono::system_clock::now();
	// fill step lineBuff with dynLineBuff
	float sum = dynLBS.size() / (float)((LINE_BUFF_SIZE_AS - 2) / 2);
	int j = 1; float overflow = 0; int overflowAmt = 0;
	bool sumUnderOne = false; bool overflowNow = false;

	// handle sub orbital trajectories
	if (sum < 1) {
		sumUnderOne = true;
		lB_size_actual = dynLBS.size();
	}
	else {
		lB_size_actual = -1;
	}

	// fill loop, divides dynLBS evenly depending on line buff size
	for (int k = 0; k < dynLBS.size(); k++) {
		overflowNow = false;
		while (!sumUnderOne && ((k - overflowAmt) % (int)sum) != 0 && k < dynLBS.size()) {
			k++;
		}
		overflow += sum - (int)sum;
		if (overflow > 1) {
			k++; overflowNow = true; overflowAmt++;
			overflow = overflow - (int)overflow;
		}
		if (k < dynLBS.size() && (overflowNow || !sumUnderOne) && j < LINE_BUFF_SIZE_AS - 1) { // shouldnt need last check
			lineBuff[j] = vertex_t{ {(glm::vec3)dynLBS[k], 2}, lineColor }; // should make alternate soi more green,  + glm::vec4{0,soi_list[k] * 0.7,0,0}
			lineBuff[j + 1] = lineBuff[j];
			j += 2;
		}
	}
	t4 = std::chrono::system_clock::now();
	std::chrono::duration<double> aaprelim, aascan, aafill;
	aaprelim = t2 - t1;
	aascan = t3 - t1;
	aafill = t4 - t3;



	lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[0];

	delete pvCurr;
	delete pvSC;
}

void ArtSat::chartApproach(std::vector<Mesh*> bodies, int targetID) {
	// could add all this to the probe step in traj, would be more efficient instead of running through update again

	// update state through orbital period until distance goes down and comes back up or lowest distance
	double dist = INT_MAX, dummy = 0;
	pvUnit pv = *state;
	int acc = 1;
	float timeStep = 60 * 5;

	glm::dvec3 caPos = pv.Pos;
	double caTime = 0;
	double roughEstTime = 0;

	while (acc < stat->orbitalPeriod) {
		// these udpatePV loops would greatly benefit from a reliable velocity based multiplticative factor
		updatePV(&pv, bodies, soiIdx, stateTime + acc, timeStep, dummy);

		glm::vec3 bodyPos = bodies[targetID]->getPV(stateTime + acc, false, true).Pos;
		bodyPos = -bodyPos;

		double newDist = distanceFind(pv.Pos, bodyPos);
		if (newDist < dist) {
			dist = newDist;
			roughEstTime = stateTime + acc;
		}
		acc += timeStep;
	}

	pv = *state;
	acc = 0;
	timeStep = 15;
	dist = INT_MAX;

	// would benefit from valid pv points along trajectory pre calculated?
	while (acc < (roughEstTime + (60 * 5) - stateTime)) {
		updatePV(&pv, bodies, soiIdx, stateTime + acc, timeStep, dummy);
		if (acc > (roughEstTime - (60 * 5) - stateTime)) {

			glm::vec3 bodyPos = bodies[targetID]->getPV(stateTime + acc, false, true).Pos;
			bodyPos = -bodyPos;

			double newDist = distanceFind(pv.Pos, bodyPos);
			if (newDist < dist) {
				dist = newDist;
				caPos = pv.Pos;
				caTime = stateTime + acc;
			}
		}
		acc += timeStep;
	}

	stateChange(&(caPos), &(pv.Vel));

	mk1 = new Marker(1, 1, (glm::vec3)caPos, *(bodies[soiIdx]->Pos));
	mk2 = new Marker(1, 1, (glm::vec3)(bodies[targetID]->getPV(caTime, true, false).Pos), *(bodies[soiIdx]->Pos)); // scheme changes for planet targets
	closeApproachDist = dist;
}

void ArtSat::refreshTraj(std::vector<Mesh*> bodies, double dt) {
	chartTraj(*state, bodies, dt);
	chartApproach(bodies, 4);
}


//***************************************************************************************************


std::vector <double> createGaussianKernel(int size, int stdDev) {
	// size must be odd
	if (size % 2 == 0) {
		size += 1;
	}
	std::vector <double> res(size, 0.0f);
	int mdpt = (size - 1) / 2;
	double sum = 0;
	for (int i = 0; i < mdpt + 1; i++) { // generate kernel
		res[i] = exp(-pow(mdpt - i, 2) / (2 * pow(stdDev, 2)));
		if (i != mdpt) res[size - 1 - i] = res[i];
		sum += 2 * res[i];
	}
	sum -= res[mdpt];

	for (auto& pt : res) { // normalize
		pt = pt / sum;
	}
	return res;
}

glm::dvec3 accel_comp(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt) {
	glm::dvec3 res = { 0,0,0 };
	double vecRes = 0;

	pvUnit soi = bodies[soiIdx]->getPV(dt, false, true);
	if (bodies[soiIdx]->isMoon) {
		soi = bodies[soiIdx]->gravSource->getPV(dt, false, true) + bodies[soiIdx]->getPV(dt, false, true);
	}
	pvUnit pvToSun = pv + soi;

	std::vector<double> str;
	for (int i = 0; i < bodies.size(); i++) {
		pvUnit pvToBody = pvToSun - bodies[i]->getPV(dt, false, true); // need mod for moons
		if (bodies[i]->isMoon) {
			pvToBody = pvToSun - (bodies[i]->gravSource->getPV(dt, false, true) + bodies[i]->getPV(dt, false, true));
		}
		glm::dvec3 acc = onePN(pvToBody, bodies[i]->mass);
		str.push_back(vecMag(acc));
		vecRes += vecMag(acc);
		res += acc;
	}

	for (auto& val : str) { // gives percentage of effort for each body for soi detection
		val /= vecRes;
	}
	return res;
}

pvUnit RK4(pvUnit pv, double M, double dt) {
	// coupled runge kutta 4th order, dt is the step size h
	double flipper = 1; // handle reverse time
	if (dt < 0) {
		flipper = -1;
		//dt = -dt;
	}
	glm::dvec3 k1 = flipper * pv.Vel;
	glm::dvec3 l1 = onePN({pv.Pos, k1}, M); // instantaneous acceleration, compare with bodies

	glm::dvec3 r2 = pv.Pos + ((dt / 2) * k1);
	glm::dvec3 k2 = flipper * pv.Vel + ((dt / 2) * l1); // also v2
	glm::dvec3 l2 = onePN(pvUnit{ r2, k2 }, M);

	glm::dvec3 r3 = pv.Pos + ((dt / 2) * k2);
	glm::dvec3 k3 = flipper * pv.Vel + ((dt / 2) * l2); // also v3
	glm::dvec3 l3 = onePN(pvUnit{ r3, k3 }, M);

	glm::dvec3 r4 = pv.Pos + (dt * k3);
	glm::dvec3 k4 = flipper * pv.Vel + (dt * l3); // also v4
	glm::dvec3 l4 = onePN(pvUnit{ r4, k4 }, M);

	return pvUnit{ (dt / 6) * (k1 + (k2 * 2.0) + (2.0 * k3) + k4) , (dt / 6) * (l1 + (l2 * 2.0) + (2.0 * l3) + l4) };
}

pvUnit leapfrog(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep) {
	// second order leapfrog algorithm (velocity-verlet)
	pvUnit dummyPV = pv;

	// run oneP PN and sum for each body (pv relative)

	glm::dvec3 halfVel = pv.Vel + (accel_comp(pv, bodies, soiIdx, dt) * (timeStep / 2));

	dummyPV.Pos = pv.Pos + (halfVel * timeStep);
	dummyPV.Vel = halfVel + (accel_comp({ dummyPV.Pos, halfVel }, bodies, soiIdx, dt) * (timeStep / 2));

	return dummyPV - pv;
}


glm::dvec3 onePN(pvUnit pv, double M) {
	// 1PN approximation
	double c2 = 299792458; c2 = c2 * c2; // m/s
	double G = 6.6743 * glm::pow(10, -11); // N * m^2 * kg^-2
	pv.Pos *= 1000; pv.Vel *= 1000; // values in meters
	double magR = vecMag(pv.Pos);
	double magV = vecMag(pv.Vel);

	double gmOverR3 = -(G * M) / (magR * magR * magR); // conservative force, easily time reversible, other acc factors aren't
	glm::dvec3 accNewt = gmOverR3 * pv.Pos;

	glm::dvec3 accRel = accNewt * (-(magV * magV) + ((double)(4 * G * M) / magR));
	accRel /= c2;

	glm::dvec3 accCross = -gmOverR3 * (4 / c2) * (glm::dot(pv.Pos, pv.Vel) * pv.Vel);

	glm::dvec3 accSelf = -gmOverR3 * (3 / c2) * (magV * magV) * pv.Pos;

	accNewt += accRel + accCross + accSelf;
	accNewt /= 1000; // back to km
	return accNewt; // supposed to be small
}

double vecMag(glm::dvec3 vec) { // glm has built-in length() fxn
	double mag = (vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z);
	return sqrt(mag);
}

float getAngle(glm::vec3 v1, glm::vec3 v2) {
	return acosf(glm::dot(v1, v2) / (vecMag(v1) * vecMag(v2)));
}

double totalEnergy(pvUnit pv, double M, double *KE, double *PE) {
	double G = 6.6743 * glm::pow(10, -14); // N * km^2 * kg^-2
	double r = glm::length(pv.Pos);
	double v2 = glm::dot(pv.Vel, pv.Vel);
	double U = -G * M / r;  // Newtonian potential energy
	double T = 0.5 * v2; // Kinetic energy
	*KE = T; *PE = U; // export individual energies
	return T + U;  // Total energy should be roughly constant
}


// update pv with 1PN approximation at specific time dt (UTC) pv units should be in km, s (NOT state changed)
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiIndex, double dt, float timeStep, double &E0) {


	pvUnit change = leapfrog(*pv, bodies, soiIndex, dt, (double)timeStep);

	*pv = *pv + change; // return to local pv
}

int gSmooth(std::vector<double>* kernel, std::vector<glm::dvec3>* noisy, std::vector<glm::dvec3>* smooth) {
	// init
	glm::dvec3 dummy( 0.0, 0.0, 0.0 );
	if (noisy->size() < kernel->size()) return 1;

	// utilize kernel 
	for (int i = 0; i < kernel->size(); i++) {
		glm::dvec3 obj = noisy->at(noisy->size() - kernel->size() + i);
		obj *= (*kernel)[i];
		dummy += obj;
	}
	smooth->push_back(dummy);
	return 0; 
}

void simCartToSph(glm::dvec3& cart) {
	// swap sim values
	double dummy = cart[0];
	cart[0] =  sqrt(pow(cart[0], 2) + pow(cart[1], 2) + pow(cart[2], 2));
	cart[2] = acos(cart[2] / cart[0]);
	cart[1] = (cart[1] / abs(cart[1])) * acos(dummy / sqrt(pow(dummy, 2) + pow(cart[1], 2)));
	cart[1] += 3 * glm::pi<float>() / 2;
	dummy = cart[1];
	cart[1] = cart[2];
	cart[2] = dummy;
}


/*
std::fstream dataStream("testDataSix.txt", std::fstream::in | std::fstream::out | std::fstream::trunc); // can read and write
if (!dataStream) {
	std::cout << "File failed to open / create\n";
	dataStream.clear();
	return 0;
}
const int stringSize = 1000000;
char dBoS[stringSize];
int m = 0;
for (int i = 0; i < dynLBS.size(); i++) {
	m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBS[i].x);
	m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBS[i].y);
	m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBS[i].z);

}

/*
m += snprintf(dBoS + m, stringSize - m, "\n");
for (int i = 50; i < satEnergy.size() - 50; i++) {
	m += snprintf(dBoS + m, stringSize - m, "%9.9f,", satEnergy[i]);
	m += snprintf(dBoS + m, stringSize - m, "%9.9f,", satT[i]);
	m += snprintf(dBoS + m, stringSize - m, "%9.9f,", satU[i]);
}

dataStream.write(dBoS, m);
dataStream.clear();
dataStream.close();
	/*
	std::vector <glm::vec3> earthSPICE;
	std::vector <glm::vec3> earthSim;
	std::vector <glm::vec3> orbitWorld;
	std::vector <glm::vec3> orbitSim;

	pvUnit* dummy = new pvUnit(bodies[soiIndex]->getPV(itTime));
	while (itTime < dt + (60 * 60 * 12)) {

		earthSPICE.push_back(dummy->Pos);
		stateChange(&dummy->Pos, &dummy->Vel);
		earthSim.push_back(dummy->Pos);

		updatePV(pvCurr, bodies, soiIndex, itTime, min / ratio);
		itTime += min / ratio;

		orbitWorld.push_back(pvCurr->Pos);
		*pvSC = *pvCurr;
		stateChange(&pvSC->Pos, &pvSC->Vel);
		orbitSim.push_back(pvSC->Pos);

		*dummy = pvUnit(bodies[soiIndex]->getPV(itTime));
	}

	std::fstream dataStream("testDataOne.txt", std::fstream::in | std::fstream::out | std::fstream::trunc); // can read and write
	if (!dataStream) {
		std::cout << "File failed to open / create\n";
		dataStream.clear();
		return 0;
	}

	// assemble string
	const int stringSize = 100000;
	char dBeSPICE[stringSize];
	char dBeSim[stringSize];
	char dBoW[stringSize];
	char dBoS[stringSize];
	int j = 0, k = 0, l = 0, m = 0;
	for (int i = 0; i < earthSPICE.size(); i++) {

		 // should do distance to center instead of coords
		j += snprintf(dBeSPICE + j, stringSize - j, "%.2f,", earthSPICE[i].x);
		j += snprintf(dBeSPICE + j, stringSize - j, "%.2f,", earthSPICE[i].y);
		j += snprintf(dBeSPICE + j, stringSize - j, "%.2f,", earthSPICE[i].z);

		k += snprintf(dBeSim + k, stringSize - k, "%.5f,", earthSim[i].x);
		k += snprintf(dBeSim + k, stringSize - k, "%.5f,", earthSim[i].y);
		k += snprintf(dBeSim + k, stringSize - k, "%.5f,", earthSim[i].z);


		l += snprintf(dBoW + l, stringSize - l, "%.2f,", orbitWorld[i].x);
		l += snprintf(dBoW + l, stringSize - l, "%.2f,", orbitWorld[i].y);
		l += snprintf(dBoW + l, stringSize - l, "%.2f,", orbitWorld[i].z);

		m += snprintf(dBoS + m, stringSize - m, "%.5f,", orbitSim[i].x);
		m += snprintf(dBoS + m, stringSize - m, "%.5f,", orbitSim[i].y);
		m += snprintf(dBoS + m, stringSize - m, "%.5f,", orbitSim[i].z);

	}
	sprintf(dBeSPICE, "\n\n");
	sprintf(dBeSim, "\n\n");
	sprintf(dBoW, "\n\n");
	sprintf(dBoS, "\n\n");
	dataStream.write(dBeSPICE, j + 2);
	dataStream.write(dBeSim, k + 2);
	dataStream.write(dBoW, l + 2);
	dataStream.write(dBoS, m + 2);
	std::cout << "files filled\n";
	dataStream.clear();
	dataStream.close();
	*/

