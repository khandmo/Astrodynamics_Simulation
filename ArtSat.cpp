#include "ArtSat.h"

pvUnit RK4(pvUnit pv, double M, double dt);
pvUnit leapfrog(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep);
glm::dvec3 onePN(pvUnit pv, double M);
glm::dvec3 accel_comp(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt);
double vecMag(glm::dvec3 vec);
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiID, double dt, float timeStep, double &E0);

std::vector <double> createGaussianKernel(int size, int stdDev);
int gSmooth(std::vector<double>* kernel, std::vector<pvUnit>* noisy, std::vector<pvUnit>* smooth);
float getAngle(glm::vec3 v1, glm::vec3 v2);
void simCartToSph(glm::dvec3& cart);

double totalEnergy(pvUnit pv, double M, double* KE, double* PE);

ArtSat::ArtSat() {

	// init shader device, will need to get uniform for camera info to render
	ArtSat::pathDevice = geom_shdr_lines_init_device();

	apoapsis = new poi;
	periapsis = new poi;
	stat = new stats;

	state = new pvUnit;
	stateButChanged = new pvUnit;

	
	stateTime = 0;
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

		char* saveName = new char;
		strcpy(saveName, "launch\0");
		maneuvers.push_back({ pv, pv, dt, "launch", "orbital insertion"});

		// chart trajectory
		// char traj in thread
		chartTraj(pv, bodies, dt);
		chartApproach(bodies, 4);
	}
	return 1;
}

// should perform this on a copy of the probe first and when save is triggered splice into System save
void ArtSat::ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, std::atomic<bool> &stop, double dt, const char name[30], const char desc[30]) {
	threadStop = &stop;
	// might need to check for past-orbit nodes
	int i = 0;
	while (1) { // is this bad code?
		if (lBTime[i] > dt) {
			i--;
			break;
		}
		i++;
	}

	int timeTill = std::max(dt - lBTime[i], (double)0);
	*state = lineBuff[2 * i];
	invStateChange(&state->Pos, &state->Vel);
	stateTime = lBTime[i];

	int acc = 0; double dummy = 0;
	while (acc < timeTill) { // would love to set up a thread to do this in the back and provide a 30 update in the meantime
		updatePV(state, bodies, soiIdx, stateTime + ++acc, 1, dummy);
	}
	pvUnit origState = *state;

	simCartToSph(state->Vel);
	state->Vel += deltaV; // add spherically
	state->Vel = sphToCart(state->Vel);
	state->Vel = { -state->Vel.x, state->Vel.z, state->Vel.y };

	char* saveName = new char[strlen(name) + 1];
	strncpy(saveName, name, strlen(name) + 1);
	saveName[strlen(saveName) - 1] = '\0';

	maneuvers.push_back({ origState, *state, dt, saveName, desc });

	*sysThread = std::async(std::launch::async, &ArtSat::chartTraj, this, *state, bodies, dt);
	chartApproach(bodies, 4);
}

void ArtSat::ArtSatUpdState(std::vector <Mesh*> bodies, double dt, int tW, double mod) {


	if (dt < maneuvers[0].time || lastEphTime != -1 && dt - lastEphTime > 60 * 60 * 24) { // mod is -1 at art sat creation
		inTime = false;
		delete satVis;
		satVis = nullptr;
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
		// altitude from earth DEBUG
		std::vector<double> distances;
		for (int i = 0; i < maneuvers.size(); i++) {
			distances.push_back(distanceFind(maneuvers[i].newState.Pos, bodies[3]->getPV(maneuvers[i].time, false, true).Pos));
		}

		// should refresh orbits at every turning point
		if (mk1 == nullptr) {
			chartApproach(bodies, 4);
		}
		else {
			mk1->corrPos = *bodies[soiIdx]->Pos;
			mk2->corrPos = *bodies[soiIdx]->Pos;
		}
		if (satVis == nullptr) {
			satVis = new Marker(1, 1, (glm::vec3)state->Pos, *(bodies[soiIdx]->Pos));
		}

		// handle state update, get close quick with orbit nodes ******** need to handle backwards maneuver travel
		int counter = dt - stateTime, manJump = 0;
		double lastTime = stateTime, dummy = 0;
		if (dt - stateTime > 60 * 100) { // arbitrary const
			manJump = 1;
			pvUnit lBpv;
			// if dt greater than last node go to it and refresh traj
			if (dt > lBTime[(LINE_BUFF_SIZE_AS / 2) - 2]) {
				lBpv = lineBuff[LINE_BUFF_SIZE_AS - 2];
				invStateChange(&lBpv.Pos, &lBpv.Vel);
				*state = lBpv; // inv state change it!
				stateTime = lBTime[(LINE_BUFF_SIZE_AS / 2) - 2];
				refreshTraj(bodies, dt);
			}

			int i = 0;
			for (i = 0; i < LINE_BUFF_SIZE_AS / 2; i++) {
				if (lBTime[i] > dt) {
					i--;
					break;
				}
			}
			lBpv = lineBuff[i * 2];
			invStateChange(&lBpv.Pos, &lBpv.Vel);
			*state = lBpv;
			lastTime = lBTime[i];
			counter = dt - lastTime;
		}
		else if (stateTime - dt > 60 * 100) { // going backwards
			manJump = 1;
			int i = 0;
			while (i < maneuvers.size() - 1) {
				if (floor(maneuvers[i].time) > floor(dt)) {
					i--;
					break;
				}
				else if (floor(maneuvers[i].time) == floor(dt)) {
					break;
				}
				i++;
			}
			*state = maneuvers[i].newState;
			lastTime = maneuvers[i].time;
			counter = std::max(dt - lastTime, (double)0);
		}
		//float timeStep = 3.0f;
		while (counter > 0) {
			updatePV(state, bodies, soiIdx, lastTime++, 1.0f, dummy);
			counter--;
		}

		*stateButChanged = *state;
		stateChange(&(stateButChanged->Pos), &(stateButChanged->Vel));
		stateTime = dt;


		ArtSat::simPos = (glm::vec3)stateButChanged->Pos + *(bodies[soiIdx]->Pos);
		satVis->fixedPos = (glm::vec3)stateButChanged->Pos;
		satVis->corrPos = *(bodies[soiIdx]->Pos);

		if (manJump) {
			// if a big jump (maneuver switch), refresh traj
			refreshTraj(bodies, dt);
		}

		// handle stats update
		stat->timeToApo = abs(apoapsis->time - (dt - stat->initTime));
		stat->timeToPeri = abs(periapsis->time - (dt - stat->initTime));
		stat->MET = dt - stat->initTime; // need start time held too
		stat->distToSoi = vecMag(state->Pos) - bodies[soiIdx]->realRadius;

		// handle model
		//glm::mat4 objModel = glm::mat4(1.0f);
		//Model = glm::translate(objModel, simPos);

		// relativize path to soi position
		for (int i = 0; i < LINE_BUFF_SIZE_AS; i++) {
			relLB[i] = { { lineBuff[i].Pos, 2 }, lineColor };
			relLB[i].pos += *(bodies[soiIdx]->Pos);
			if (mod == 1) {
				relLB[i].col = { 0.0f, 0.3f, 1.0f, 0.43f };
			}
		}
	}
}

void ArtSat::ArtSatRender(Camera* camera, Mesh lightSource) {
	if (inTime) {

		// render lineBuff
		uniform_data_t uni;
		glm::mat4 mvp = camera->cameraMatrix * glm::mat4(1.0); // mult 4x4 glm - non zero, should check shader
		uni.mvp = &mvp[0][0];
		glm::vec4 vpt = glm::vec4(0, 0, (float)camera->width, (float)camera->height);
		uni.viewport = &vpt.z;
		glm::vec2 aa_radii = glm::vec2(2.0f, 2.0f);
		uni.aa_radius = &aa_radii.x;

		geom_shdr_lines_update(pathDevice, &relLB,
			LINE_BUFF_SIZE_AS, sizeof(vertex_t), &uni);

		int buffSize = LINE_BUFF_SIZE_AS;
		if (lB_size_actual != -1)
			buffSize = lB_size_actual;
		geom_shdr_lines_render(pathDevice, buffSize);

		satVis->MarkerRender(camera);
		mk1->MarkerRender(camera);
		mk2->MarkerRender(camera);
	}
}

ArtSat::~ArtSat() {
	// delete pvunit, lineBuff, obj Mesh, and locked pos and vel init
	delete state;
	delete stateButChanged;
	delete apoapsis;
	delete periapsis;
	delete stat;
	delete prevPV;
	delete mk1;
	delete mk2;
	geom_shdr_lines_term_device((void**)(&pathDevice));
}

void ArtSat::chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt) {
	while ((threadStop == nullptr || !(*threadStop)) && !fxnStop) {
		// init vars
		double itTime = dt;
		float min = 60.0f;
		int flpr = -1;

		pvUnit* pvCurr = new pvUnit(pv);
		pvUnit* pvSC = new pvUnit(pv);

		stateChange(&(pvSC->Pos), &(pvSC->Vel));
		lineBuff[0] = *pvSC;

		std::vector <pvUnit> dynLBN, dynLBS;
		std::vector <double> dynTime;
		dynBuff = &dynLBS;
		dynTimes = &dynTime;

		bool close = false, dir = false, dir2 = false;
		double lastToPV, toPV = 0;
		int lastItTime = 0; double lastVelocity = 0, velocity = 0;
		double factor = 1;
		int tp = 0; // # of turning points
		int iterations = 0; // total iteration count
		int ratio = 20;
		poi apo{ {0,0,0}, 0 }, peri{ {0,0,0}, 0 }; // turning point init

		double* T = new double, * U = new double;
		double energy_0 = totalEnergy(*pvCurr, bodies[soiIdx]->mass, T, U);

		// init kernel
		std::vector<double> gKernel = createGaussianKernel(101, 15);


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
					dummy = dynLBN[i].Pos;
					size_t dummier = dynLBN.size() - 1 - i;
					dynLBN[i] = dynLBN[dummier];
					dynLBN[dummier].Pos = dummy;
				}
			}

			// a & b check for turning points
			double a = abs(getAngle(pvCurr->Pos, pvCurr->Vel) - (glm::pi<double>() / 2));

			// hold times of updates
			if (flpr == 1) {
				dynTime.push_back(itTime);
			}
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
			dynLBN.push_back(*pvSC);

			// smoothing fxn
			gSmooth(&gKernel, &dynLBN, &dynLBS); // first pt of dynLBS is 0


			// general handling
			if (dynLBS.size() > 2) {
				c = vecMag(dynLBS[dynLBS.size() - 1].Pos);
				lastToPV = toPV;
				toPV = distanceFind(lineBuff[0].Pos, dynLBS[dynLBS.size() - 1].Pos);
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

					if (apo.time == 0 || vecMag(apo.Pos) < vecMag(dynLBS[dynLBS.size() - 1].Pos)) {
						apo.Pos = dynLBS[dynLBS.size() - 1].Pos; apo.time = itTime - dt;
					}

					if (peri.time == 0 || vecMag(peri.Pos) > vecMag(dynLBS[dynLBS.size() - 1].Pos)) {
						peri.Pos = dynLBS[dynLBS.size() - 1].Pos; peri.time = itTime - dt;
					}
				}

				if (c < bodies[soiIdx]->radius || (tp >= 3 && toPV > lastToPV) || iterations > 5000) // arbitrary constant
					close = true;
			}

		}
		stat->apoapsis = (vecMag(apo.Pos) / (LENGTH_SCALE)) - bodies[soiIdx]->realRadius;
		stat->periapsis = (vecMag(peri.Pos) / (LENGTH_SCALE)) - bodies[soiIdx]->realRadius;
		stat->orbitalPeriod = itTime - dt; // in seconds

		delete T; delete U;
		*apoapsis = apo;
		*periapsis = peri;

		fillBuff(&dynLBS, &dynTime);

		dynBuff = nullptr;
		dynTimes = nullptr;

		delete pvCurr;
		delete pvSC;
		fxnStop = true;
	}
	fxnStop = false;
}

void ArtSat::fillBuff(std::vector<pvUnit>* dynBuff, std::vector<double>* dynTime) {

	if (dynBuff == nullptr)
		return;

	// fill step lineBuff with dynLineBuff
	float sum = dynBuff->size() / (float)((LINE_BUFF_SIZE_AS - 2) / 2);
	int j = 1; float overflow = 0; int overflowAmt = 0;
	bool sumUnderOne = false; bool overflowNow = false;

	// handle sub orbital trajectories
	if (sum < 1) {
		sumUnderOne = true;
		lB_size_actual = dynBuff->size();
	}
	else {
		lB_size_actual = -1;
	}

	// fill loop, divides dynLBS evenly depending on line buff size
	for (int k = 0; k < dynBuff->size(); k++) {
		overflowNow = false;
		while (!sumUnderOne && ((k - overflowAmt) % (int)sum) != 0 && k < dynBuff->size()) {
			k++;
		}
		overflow += sum - (int)sum;
		if (overflow > 1) {
			k++; overflowNow = true; overflowAmt++;
			overflow = overflow - (int)overflow;
		}
		if (k < dynBuff->size() && (overflowNow || !sumUnderOne) && j < LINE_BUFF_SIZE_AS - 1) { // shouldnt need last check
			lineBuff[j] = { (*dynBuff)[k] }; 
			lineBuff[j + 1] = lineBuff[j];
			lBTime[j / 2] = (*dynTime)[k];
			j += 2;
		}
	}


	lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[0];
	// ensure closure only when the orbits should close
	if (distanceFind(lineBuff[LINE_BUFF_SIZE_AS - 1].Pos, lineBuff[LINE_BUFF_SIZE_AS - 2].Pos) >
		(2 * distanceFind(lineBuff[LINE_BUFF_SIZE_AS - 2].Pos, lineBuff[LINE_BUFF_SIZE_AS - 3].Pos))) {
		lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[LINE_BUFF_SIZE_AS - 2];
	}

}

void ArtSat::chartApproach(std::vector<Mesh*> bodies, int targetID) {

	double dist = INT_MAX, dummy = 0;
	int closestNode = 0;


	// loop through LB time and find closest pt in orbit to planet, then march back one pt and iterate to get the exact pt
	for (int i = 0; i < LINE_BUFF_SIZE_AS;) { // only searches 100 pts
		double dummyDist = distanceFind(lineBuff[i].Pos, bodies[targetID]->getPV(lBTime[i/2], true, true).Pos);
		if (dummyDist < dist) {
			dist = dummyDist;
			closestNode = i;
		}
		i += 2;
	}

	dist = INT_MAX;
	pvUnit pv = lineBuff[closestNode - 2], pvb4 = pv;
	invStateChange(&(pv.Pos), &(pv.Vel));
	double time = lBTime[(closestNode - 2) / 2];
	double timeStep = 3.0;

	while (0) { // 0 is false
		updatePV(&pv, bodies, soiIdx, time, timeStep, dummy);
		double dummyDist = distanceFind(pv.Pos, bodies[targetID]->getPV(time + timeStep, false, true).Pos);
		if (dummyDist > dist) {
			dist = dummyDist;
			break;
		}
		dist = dummyDist;
		time += timeStep;
		pvb4 = pv;
	}	

	mk1 = new Marker(0, 1, (glm::vec3)pvb4.Pos, *(bodies[soiIdx]->Pos));
	mk2 = new Marker(0, 1, (glm::vec3)(bodies[targetID]->getPV(time, true, false).Pos), *(bodies[soiIdx]->Pos)); // scheme changes for planet targets
	closeApproachDist = dist;
}

void ArtSat::refreshTraj(std::vector<Mesh*> bodies, double dt) {
	chartTraj(*state, bodies, dt);
	chartApproach(bodies, 4);
}

// integrate two traj's and add maneuver to bridge the gap
void ArtSat::solveManeuver(std::vector<Mesh*> bodies, char* name, pvUnit pv1, pvUnit pv2, double t1, double t2) {
	std::vector<pvUnit> pvAL, pvBL;
	pvUnit pvA = pv1, pvB = pv2;
	double dummy, timeStep = 3;

	for (int i = 0; i < t2 - t1; ) { // compile trajectories
		pvAL.push_back(pvA);
		pvBL.push_back(pvB);
		updatePV(&pvA, bodies, soiIdx, t1 + i, timeStep, dummy);
		updatePV(&pvB, bodies, soiIdx, t2 - i, -timeStep, dummy);
		i += timeStep; 
	}
	for (int i = 0; i < pvBL.size() / 2; i++) { // flip backprop
		pvUnit d2 = pvBL[i];
		pvBL[i] = pvBL[pvBL.size() - 1 - i];
		pvBL[pvBL.size() - 1 - i] = d2;
	}

	// find intersect, add maneuver to list
	int i = 0; double dist = INT_MAX;
	while (1) {
		double trajDist = distanceFind(pvAL[i].Pos, pvBL[i].Pos);
		if (trajDist > dist) {
			break;
		}
		dist = trajDist;
		i++;
	}
	char* saveName = new char[strlen(name) + 1];
	strncpy(saveName, name, strlen(name) + 1);
	maneuvers.push_back({ pvAL[i], pvBL[i],  t1 + (3 * i), saveName, "?" });
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

	std::vector<double> str; // hold influence of each body
	for (int i = 0; i < bodies.size(); i++) {
		pvUnit pvToBody = pvToSun;
		if (i != 0) {
			pvToBody = pvToBody - bodies[i]->getPV(dt, false, true);
		}
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

int gSmooth(std::vector<double>* kernel, std::vector<pvUnit>* noisy, std::vector<pvUnit>* smooth) {
	// init
	glm::dvec3 dummyPos( 0.0, 0.0, 0.0 );
	glm::dvec3 dummyVel( 0.0, 0.0, 0.0 );
	if (noisy->size() < kernel->size()) return 1;

	// utilize kernel 
	for (int i = 0; i < kernel->size(); i++) {
		glm::dvec3 objPos = noisy->at(noisy->size() - kernel->size() + i).Pos;
		glm::dvec3 objVel = noisy->at(noisy->size() - kernel->size() + i).Vel;
		objPos *= (*kernel)[i];
		objVel *= (*kernel)[i];
		dummyPos += objPos;
		dummyVel += objVel;
	}
	smooth->push_back({dummyPos, dummyVel});
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

