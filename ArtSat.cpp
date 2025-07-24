#include "ArtSat.h"

// integrator
pvUnit RK4(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep);
// integrator (sympletic)
pvUnit leapfrog(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep);
// integrator (sympletic)
pvUnit yoshida(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep);
// 2-step RK4 integrator for error control
pvUnit composed_RK4(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep);
// post newtonian acceleration calculator
glm::dvec3 onePN(pvUnit pv, double M);
// total acceleration computer
glm::dvec3 accel_comp(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt);
// finds vector magnitude
double vecMag(glm::dvec3 vec);
// updates state
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiID, double dt, float timeStep, double &E0);



// detects sat current soi
bool soiDetector(std::vector<Mesh*> bodies, glm::dvec3 satPos, double dt, int* soiIdx, int specifyCheck);
// returns pv wrt relativeBody assuming it's input from currBody
pvUnit translatePV(pvUnit pv, std::vector<Mesh*> bodies, double dt, int currBody, int relativeBody);
// returns filled and normalized gaussian kernel
std::vector <double> createGaussianKernel(int size, int stdDev);
// uses kernel on noisy set and outputs to smooth set
int gSmooth(std::vector<double>* kernel, std::vector<pvUnit>* noisy, std::vector<pvUnit>* smooth);
// cartesian to spherical coord calculator
void simCartToSph(glm::dvec3& cart);
// returns total energy, outputs kinetic and potential to ptrs
double totalEnergy(pvUnit pv, double M, double* KE, double* PE);
// calculates adapting time factor for efficient orbit drawing
double timeFactorCalc(pvUnit* pv, std::vector<Mesh*> bodies, int soiID);
// determines when orbits should close based on angles
bool angleCalc(pvUnit* pvCurr, std::vector<double>& angleList, int iterations, double& firstAngle, double& lastAngle, int& lastQuad, int& quadChange, int& angleAxis, bool& lostAxis, bool& cw);
// grow buffers by 1 section
void buffAppend(pvUnit* &lB, vertex_t* &relLB, double* &lBT, std::vector<std::pair<int, int>>& lineBuffSect);

ArtSat::ArtSat() {

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
		
		threadStop->store(true);
		while (sysThread->valid()) { // make sure thread is dead
			if (sysThread->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
				sysThread->get();
			}
		}
		threadStop->store(false);

		soiIdx = soiIndex;

		if (prevPV == nullptr) {
			prevPV = new pvUnit(pv);
			lineBuffSect.push_back({ soiIdx, LINE_BUFF_SIZE_AS });
			maneuvers.push_back({ pv, pv, dt, "launch", "orbital insertion", soiIndex });
		}else {
			*prevPV = pv;
			lineBuffSect.clear();
			maneuvers.clear();
			lineBuffSect.push_back({ soiIdx, LINE_BUFF_SIZE_AS });
			maneuvers.push_back({ pv, pv, dt, "launch", "orbital insertion", soiIndex });
		}

		// add pos as poi to carry
		*ArtSat::state = pv;

		pvUnit* pvSC = new pvUnit(pv);
		stateChange(&(pvSC->Pos), &(pvSC->Vel));

		*ArtSat::stateButChanged = *pvSC;
		ArtSat::stateTime = dt;

		stat->initTime = dt;

		planning = true;
		// chart trajectory
		// char traj in thread
		*sysThread = std::async(std::launch::async, &ArtSat::chartTraj, this, *state, bodies, dt, -1);

		planning = false;
	}
	return 1;
}

// should perform this on a copy of the probe first and when save is triggered splice into System save
void ArtSat::ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, double dt, const char name[30], const char desc[30]) {
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
	float timeStep = std::max(1.0f, (float)stat->orbitalPeriod / 100000);
	while (acc < timeTill) { // would love to set up a thread to do this in the back and provide a 30 update in the meantime
		acc += (int)timeStep;
		updatePV(state, bodies, soiIdx, stateTime + acc, 1, dummy);
	}
	pvUnit origState = *state;

	simCartToSph(state->Vel);
	state->Vel += deltaV; // add spherically
	state->Vel = sphToCart(state->Vel);
	state->Vel = { -state->Vel.x, state->Vel.z, state->Vel.y };
	
	*stateButChanged = *state;
	stateChange(&stateButChanged->Pos, &stateButChanged->Vel);

	char* saveName = new char[strlen(name) + 1];
	strncpy(saveName, name, strlen(name) + 1);
	saveName[strlen(saveName)] = '\0';

	maneuvers.push_back({ origState, *state, dt, saveName, desc, soiIdx });

	*sysThread = std::async(std::launch::async, &ArtSat::chartTraj, this, *state, bodies, dt, -1);
}

void ArtSat::ArtSatUpdState(std::vector <Mesh*> bodies, double dt, int &tW, double mod) {
	if (tW != 15) {
		float timeDifference = dt - maneuvers[maneuvers.size() - 1].time; // oughta be positive
		// if i put sat death on last maneuver then i can check here for inTime
		// if EOM found it should update the save file, the second argument here just says if its been over 1 day since the last time it was updated then assume its dead
		// rigorous method would not update unless prompted and run it on the thread until it's eom or otherwise
		if (dt + 5 < maneuvers[0].time || lastEphTime != -1 && abs(dt - lastEphTime) > 60 * 60 
			|| maneuvers[maneuvers.size() - 1].newState == pvUnit{ glm::dvec3(0,0,0), glm::dvec3(0,0,0) } && dt > maneuvers[maneuvers.size() - 1].time) { // mod is -1 at art sat creation
			inTime = false;
			simPos = glm::vec3(0,0,0);
			delete satVis;
			satVis = nullptr;
			return;
		}
		else {
			inTime = true;
		}

		// check if maneuver scheduled and execute
		if (mod != -1 && !isCopy) {
			for (int i = 0; i < maneuvers.size(); i++) {
				// if hit
				if (abs(maneuvers[i].time - dt) < 1) { // if maneuver is now

					if (tW < 15) {
						*state = maneuvers[i].origState;
						
					}
					else {
						*state = maneuvers[i].newState;
					}

					stateTime = dt;
					soiIdx = maneuvers[i].soi;

					chartTraj(*state, bodies, dt, -1);
					if (targStat != nullptr)
						chartApproach(bodies, targStat->targetIdx);
					return;
				}
				// if blow past, these go through each prev, not trigger on correct
				else if (tW > 15 && dt > maneuvers[i].time && lastManIdx < i) {
					*state = maneuvers[i].newState;
					lastManIdx = i;
					soiIdx = maneuvers[i].soi;

					chartTraj(*state, bodies, maneuvers[i].time, -1);
					if (targStat != nullptr)
						chartApproach(bodies, targStat->targetIdx);
					return;
				}
				else if (tW < 15 && maneuvers[i].time > dt && lastManIdx > i) {
					*state = maneuvers[i].origState;
					lastManIdx = i;
					soiIdx = maneuvers[i].soi;

					chartTraj(*state, bodies, maneuvers[i].time, -1);
					if (targStat != nullptr)
						chartApproach(bodies, targStat->targetIdx);
					return;
				}
			}
		}


		if (inTime) {
			mtxSat->lock();
			// check if reached unfavorable position
			if (!escaping && (abs(vecMag(state->Pos)) > pow(10, 20) || abs(vecMag(state->Pos)) < bodies[soiIdx]->realRadius)) {
				inTime = false;
				if (strcmp(maneuvers[maneuvers.size() - 1].name, "EOM") == 0)
					maneuvers.push_back({ *state, *state, stateTime, "EOM", "an error or death has likely occured", soiIdx });
				mtxSat->unlock();
				return;
			}


			if (mod == 1 && lineColor != glm::vec4{ 0.0f, 0.3f, 1.0f, 0.43f }) {
				lineColor = { 0.0f, 0.3f, 1.0f, 0.43f };
			}
			
			// update target info, markers if changed
			if (targStat != nullptr && (int)lastTargIdx != targStat->targetIdx) { // mk1 == nullptr && 
				lastTargIdx = targStat->targetIdx;
				chartApproach(bodies, targStat->targetIdx);
			}
			else if (mk1 != nullptr) {
				mk1->corrPos = *bodies[soiIdx]->Pos;
				if (targStat != nullptr && bodies[targStat->targetIdx]->isMoon)
					mk2->corrPos = *bodies[targStat->targetIdx]->gravSource->Pos;
			}
			if (satVis == nullptr) {
				satVis = new Marker(1, 1, (glm::vec3)stateButChanged->Pos, *bodies[soiIdx]->Pos);
			}
			// refresh orbit at soi change
			int lastSoi = soiIdx;
			if (!isCopy && soiDetector(bodies, state->Pos, stateTime, &soiIdx, -1)) {
				*state = translatePV(*state, bodies, stateTime, lastSoi, soiIdx);
				mtxSat->unlock();
				refreshTraj(bodies, stateTime);
				mtxSat->lock();
			}

			// handle state update on time jump, get close quick with orbit nodes
			int manJump = 0;
			double lastTime = stateTime, dummy = 0;
			if (tW > tW_thresh) { // arbitrary const (dt - stateTime > (stat->orbitalPeriod / 20)), tW greater than 2000x
				manJump = 1;
				pvUnit lBpv;
				// if dt greater than last node go to it and refresh traj
				/*
				while (dt > lBTime[(LINE_BUFF_SIZE_AS / 2) - 2]) {
					lBpv = lineBuff[LINE_BUFF_SIZE_AS - 2];
					invStateChange(&lBpv.Pos, &lBpv.Vel);
					*state = lBpv; 
					stateTime = lBTime[(LINE_BUFF_SIZE_AS / 2) - 2];
					refreshTraj(bodies, dt);
				}
				*/
				std::vector<double> distances, times;
				bool quit = false;
				double hDist = INT_MAX;
				int i = 0, timeSln, posSln;
				for (i = 0; i < LINE_BUFF_SIZE_AS / 2; i++) {
					times.push_back(lBTime[i] - stateTime);
					if (i == (LINE_BUFF_SIZE_AS / 2) - 1 && !quit && lineBuffSect.size() > 0) {
						timeSln = i;
						tW = tW_thresh - 1;
						break;
					}
					if (lBTime[i] > stateTime && !quit) {
						timeSln = i;
						quit = true;
						if (i != 0)
							timeSln--;
					}
				}
				if (i < 0) i = 0; // error bandaid
				if (i >= LINE_BUFF_SIZE_AS * 50) i = LINE_BUFF_SIZE_AS - 1;
				i = timeSln;
				lBpv = lineBuff[i * 2];
				invStateChange(&lBpv.Pos, &lBpv.Vel);

				*state = lBpv;	
				lastTime = lBTime[i];
			}
			else if (tW < 6) { // going backwards
				manJump = 1;
				int j = 0;
				while (j < maneuvers.size() - 1) {
					if (floor(maneuvers[j].time) > floor(dt)) {
						j--;
						break;
					}
					else if (floor(maneuvers[j].time) == floor(dt)) {
						break;
					}
					j++;
				}
				*state = maneuvers[j].newState;
				lastTime = maneuvers[j].time;
				soiIdx = maneuvers[j].soi;
			}
			// once have closest orbit node, integrate to curr time
			float timeStep = std::max(1.0f, (float)stat->orbitalPeriod / 100000);
			if (tW >= tW_thresh || dt - lastTime > 60 * 60 * 24) timeStep = stat->orbitalPeriod / 50000; // should use thread past this threshold, greater than 1000x
			if (tW < 15) timeStep = -timeStep;

			while (lastTime < dt && tW <= tW_thresh) { // less than or equal to 2000x @ thresh 24
				pvUnit last = *state;
				updatePV(state, bodies, soiIdx, lastTime + timeStep, timeStep, dummy);
				

				if (abs(vecMag(state->Pos)) < bodies[soiIdx]->realRadius + 100) {
					inTime = false;
					maneuvers.push_back({ last, {glm::dvec3(0,0,0), glm::dvec3(0,0,0)}, lastTime, "EOM", "Landed/Crashed", soiIdx});
					mtxSat->unlock();
					return;
				}

				lastTime += timeStep;
			}


			*stateButChanged = *state;
			stateChange(&(stateButChanged->Pos), &(stateButChanged->Vel));
			stateTime = dt;


			ArtSat::simPos = (glm::vec3)stateButChanged->Pos + *(bodies[soiIdx]->Pos);
			if (!isCopy) {
				satVis->fixedPos = (glm::vec3)stateButChanged->Pos;
				satVis->corrPos = *(bodies[soiIdx]->Pos);
			}
			else { // just update corrPos
				satVis->corrPos = *(bodies[soiIdx]->Pos);
			}

			if ((sysThread->valid() && sysThread->wait_for(std::chrono::seconds(0)) == std::future_status::ready 
				|| !sysThread->valid()) && dt > lBTime[(lineBuffSize / 2) - 10] && lineBuffSect.size() == 1) {
				// if a big jump (maneuver switch), refresh traj
				mtxSat->unlock();
				refreshTraj(bodies, dt);
				mtxSat->lock();
			}
			mtxSat->unlock();
		}
	}

	if (inTime) {
		mtxSat->lock();
		// handle stats update
		stat->timeToApo = abs(apoapsis->time - (dt - stat->initTime));
		stat->timeToPeri = abs(periapsis->time - (dt - stat->initTime));
		stat->MET = dt - stat->initTime; // need start time held too
		stat->distToSoi = vecMag(state->Pos) - bodies[soiIdx]->realRadius;
		if (targStat != nullptr) {
			targStat->timeToCloseAppr = closeApproachTime - dt;
			targStat->timeToCapture = captureTime - dt;
		}

		// relativize path to soi position and set fill amt

		
		// i - lB section, j - idx per section, k - relLB idx, l - lineBuff idx
		int k = 0, l = 0;
		for (int i = 0; i < lineBuffSect.size(); i++) {
			glm::vec4 buffColor = lineColor;
			if (i != 0)
				buffColor = { 0.2f * i , 0.8f, 0.2f * i, 0.43f };

			// reset l before each new section
			l = 0;
			l += LINE_BUFF_SIZE_AS * i;

			for (int j = 0; j < lineBuffSect[i].second; j++) {
				relLB[k] = { { lineBuff[l].Pos, 2 }, buffColor };
				relLB[k].pos += *(bodies[lineBuffSect[i].first]->Pos);

				if (lineBuff[l].Pos.x == lineBuff[l].Pos.y && lineBuff[l].Pos.y == lineBuff[l].Pos.z && lineBuff[l].Pos.x == 0) { // lazy fix
					relLB[k] = relLB[k - 1];
					//continue;
				}

				if (j == LINE_BUFF_SIZE_AS - 1) // makes sure the traj's don't connect
					relLB[k] = relLB[k - 1];

				k++; l++;
			}

		}
		lB_size_actual = k;
		mtxSat->unlock();
	}
}

void ArtSat::ArtSatRender(Camera* camera, Mesh lightSource) {
	if (inTime) {
		mtxSat->lock();
		double main = vecMag(relLB[0].pos) - vecMag(relLB[1].pos);
		double next = vecMag(relLB[2].pos) - vecMag(relLB[3].pos);
		double diff = main - next;
		// render lineBuff
		uniform_data_t uni;
		glm::mat4 mvp = camera->cameraMatrix * glm::mat4(1.0); // mult 4x4 glm - non zero, should check shader
		uni.mvp = &mvp[0][0];
		glm::vec4 vpt = glm::vec4(0, 0, (float)camera->width, (float)camera->height);
		uni.viewport = &vpt.z;
		glm::vec2 aa_radii = glm::vec2(2.0f, 2.0f);
		uni.aa_radius = &aa_radii.x;

		int buffSize = LINE_BUFF_SIZE_AS;
		if (lB_size_actual != -1)
			buffSize = lB_size_actual;

		geom_shdr_lines_update(pathDevice, relLB,
			buffSize, sizeof(vertex_t), &uni);
				
		geom_shdr_lines_render(pathDevice, buffSize);
		mtxSat->unlock();

		if (satVis != nullptr)
			satVis->MarkerRender(camera);
		if (mk1 != nullptr) {
			mk1->MarkerRender(camera);
			mk2->MarkerRender(camera);
		}
	}
}

ArtSat::~ArtSat() {
	if (lineBuff != nullptr) delete[] lineBuff;
	if (relLB != nullptr) delete[] relLB;
	if (lBTime != nullptr) delete[] lBTime;

	if (state != nullptr) delete state;
	if (stateButChanged != nullptr) delete stateButChanged;
	if (apoapsis != nullptr) delete apoapsis;
	if (periapsis != nullptr) delete periapsis;
	if (stat != nullptr) delete stat;
	if (targStat != nullptr) delete targStat;
	if (prevPV != nullptr) delete prevPV;
	if (satVis != nullptr) delete satVis;
	if (mk1 != nullptr) delete mk1;
	if (mk2 != nullptr) delete mk2;
	if (pathDevice != nullptr) geom_shdr_lines_term_device((void**)(&pathDevice));
}

void ArtSat::chartTraj(pvUnit pv, std::vector<Mesh*> bodies, double dt, int altSoi) {

	int chartSoi = soiIdx;
	if (altSoi != -1) chartSoi = altSoi;

	mtxSat->lock();
	if (!escaping) {
		lineBuffSect.clear();
		lineBuffSect.push_back({ soiIdx, LINE_BUFF_SIZE_AS });

		lineBuffSize = -1;
	}


	dynBuff = new std::vector <pvUnit>;
	dynBuff->reserve(10000);
	dynTimes = new std::vector <double>;
	dynTimes->reserve(10000);
	mtxSat->unlock();

	// thread change guard
	while (((threadStop == nullptr || !(*threadStop)) && !fxnStop) && dynBuff != nullptr) {
		// init vars
		double itTime = dt;
		float min = 60.0f * pow(abs(log10(bodies[chartSoi]->mass / bodies[3]->mass)) + 1, 1.5) ;
		int flpr = 1; // removed filter, made to 1

		pvUnit* pvCurr = new pvUnit(pv);
		pvUnit* pvSC = new pvUnit(pv);

		stateChange(&(pvSC->Pos), &(pvSC->Vel));

		if (!escaping) {
			lineBuff[0] = *pvSC;
			lBTime[0] = dt;
		}

		std::vector <double> dynLBN, dynPos, dynVel;
		

		bool close = false, dir = false, dir2 = false, escape = false;
		double lastToPV, toPV = 0;
		int lastItTime = 0;
		double factor = 1;
		int tp = 0; // # of turning points
		int iterations = 0; // total iteration count
		int ratio = 20;
		//if (fastChart)
			//ratio = 1;
		poi apo{ {0,0,0}, 0 }, peri{ {0,0,0}, 0 }; // turning point init
		int prevSoi = chartSoi;

		double* T = new double, * U = new double;
		double energy_0 = totalEnergy(*pvCurr, bodies[chartSoi]->mass, T, U);

		// init kernel
		std::vector<double> gKernel = createGaussianKernel(101, 5);

		std::vector<double> angleList;
		// could turn into a struct and pass into a function
		double firstAngle = 0;
		double lastAngle = 0;
		int angleAxis = -1;
		bool lostAxis = false;
		int lastQuad = -1;
		int quadChange = 0;
		bool cw = NULL, angleClose = false;

		double mRatio = bodies[chartSoi]->mass / bodies[3]->mass;
		double mFactor = pow(10.0, -log10(mRatio) * (3.0 / 7.0));
		

		// scan step
		while (!close) {
			iterations++;
			// reset to forward time after sample pts collected for smoothing
			/*
			if (dynLBN.size() > ((gKernel.size() - 1) / 2) && flpr == -1) {
				itTime = dt;
				flpr = 1;
				*pvCurr = pv;
				glm::dvec3 dummy;
				double dummy2;
				for (int i = 0; i < (dynLBN.size() - 1) / 2; i++) {
					dummy = dynLBN[i].Pos;
					size_t dummier = dynLBN.size() - 1 - i;
					dynLBN[i] = dynLBN[dummier];
					dynLBN[dummier].Pos = dummy;
				}
			}*/

			// calc angle, determine when to close orbit
			double aAngle = getAngle(pvCurr->Pos, pvCurr->Vel) - (glm::pi<double>() / 2);
			angleClose = angleCalc(pvCurr, angleList, iterations, firstAngle, lastAngle, lastQuad, quadChange, angleAxis, lostAxis, cw);
			double a = abs(aAngle);



			glm::dvec3 lastVel = pvCurr->Vel;

			// generate new prediction
			updatePV(pvCurr, bodies, chartSoi, itTime, (flpr * min * factor) / ratio, energy_0); // removed factor
			itTime += ((flpr * min * factor) / ratio);

			// hold times of updates
			if (flpr == 1) {
				dynTimes->push_back(itTime);
			}

			double b = abs(getAngle(pvCurr->Pos, pvCurr->Vel) - (glm::pi<double>() / 2));
			double c = 0; // checks distance to soi

			factor = timeFactorCalc(pvCurr, bodies, chartSoi);
			double posFactor = (vecMag(pvCurr->Pos) - bodies[chartSoi]->realRadius) * mFactor / 1000;
			dynLBN.push_back(factor);
			dynPos.push_back(posFactor);

			*pvSC = *pvCurr;
			stateChange(&(pvSC->Pos), &(pvSC->Vel));
			dynBuff->push_back(*pvSC); // change this and smooth and flpr for leapfrog/yoshida
			

			// smoothing fxn
			//gSmooth(&gKernel, &dynLBN, &dynLBS); // first pt of dynLBS is 0


			// general handling
			if (dynBuff->size() > 2) {
				c = vecMag((*dynBuff)[dynBuff->size() - 1].Pos);
				lastToPV = toPV;
				toPV = distanceFind(lineBuff[LINE_BUFF_SIZE_AS * (lineBuffSect.size() - 1)].Pos, (*dynBuff)[dynBuff->size() - 1].Pos);
				dir2 = dir;

				if (a >= b) // U growing
					dir = true;
				else
					dir = false;
			}
			

			// check for turning points / close trigger
			if (dynBuff->size() > 5) {
				if (dir != dir2 && (iterations - lastItTime) > 200) {
					tp++;
					lastItTime = iterations;

					if (apo.time == 0 || vecMag(apo.Pos) < vecMag((*dynBuff)[dynBuff->size() - 1].Pos)) {
						apo.Pos = (*dynBuff)[dynBuff->size() - 1].Pos; apo.time = itTime - dt;
					}

					if (peri.time == 0 || vecMag(peri.Pos) > vecMag((*dynBuff)[dynBuff->size() - 1].Pos)) {
						peri.Pos = (*dynBuff)[dynBuff->size() - 1].Pos; peri.time = itTime - dt;
					}
				}
				if (c < bodies[chartSoi]->radius + (50 * LENGTH_SCALE) || angleClose || iterations > 15000 || abs(vecMag(pvSC->Pos)) > 10000) // arbitrary constant,  (tp >= 3 && toPV > lastToPV)
					close = true;
				if (soiDetector(bodies, pvCurr->Pos, itTime, &prevSoi, -1)) {
					close = true;
					escape = true;
				}			
			}

		}
		// dynamically appropriate buffer sizes
		if (lineBuffSize == -1) lineBuffSize = 0;
		if (iterations * 2 < LINE_BUFF_SIZE_AS) {
			lineBuffSect[lineBuffSect.size() - 1].second = iterations * 2;
		}
		lineBuffSize += lineBuffSect[lineBuffSect.size() - 1].second;

		// set up stats for found orbit
		if (!escaping && !escape) {
			stat->apoapsis = (vecMag(apo.Pos) / (LENGTH_SCALE)) - bodies[chartSoi]->realRadius;
			stat->periapsis = (vecMag(peri.Pos) / (LENGTH_SCALE)) - bodies[chartSoi]->realRadius;
			stat->orbitalPeriod = itTime - dt; // in seconds

			*apoapsis = apo;
			*periapsis = peri;
		}
		else if (!escaping && escape)
			stat->orbitalPeriod = itTime - dt;


		// fill lineBuff from dynamic buffers
		if (!escaping)
			fillBuff(0);
		else
			fillBuff(1);

		mtxSat->lock();
		delete T; delete U;
		delete dynBuff;
		delete dynTimes;
		dynBuff = nullptr;
		dynTimes = nullptr;
		mtxSat->unlock();

		// check soi change during traj
		//soiHandle(bodies, dt);
		
		if (escape && !planning) {
			// add another section to lineBuff
			// run through chartTraj again with sun soi
			// might need to modify base timeStep
			mtxSat->lock();
			buffAppend(lineBuff, relLB, lBTime, lineBuffSect);
			lineBuffSect.push_back({prevSoi, LINE_BUFF_SIZE_AS});
			pvUnit newPV = translatePV(*pvCurr, bodies, itTime, chartSoi, prevSoi);
			mtxSat->unlock();

			escaping = true;

			trajTransitions.push(chartSoi);
			chartTraj(newPV, bodies, itTime, prevSoi);
			trajTransitions.pop();
			if (trajTransitions.empty()) {
				escaping = false;
			}

		}
		

		delete pvCurr;
		delete pvSC;
		
		/*
		if (escaping) {
			std::fstream dataStream3("testDataDV.txt", std::fstream::in | std::fstream::out | std::fstream::trunc); // can read and write
			if (!dataStream3) {
				std::cout << "File failed to open / create\n";
				dataStream3.clear();
				return;
			}
			const int stringSize = 300000;
			char dBoS[stringSize];
			int m = 0;
			for (int i = 0; i < dynLBN.size(); i++) {
				m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBN[i]);
			}

			m += snprintf(dBoS + m, stringSize - m, "\n");
			for (int i = 0; i < dynPos.size(); i++) {
				m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynPos[i]);
			}
			/*
			m += snprintf(dBoS + m, stringSize - m, "\n");
			for (int i = 0; i < dynVel.size(); i++) {
				m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynVel[i]);
			}
			/*
			m += snprintf(dBoS + m, stringSize - m, "\n");
			for (int i = 0; i < allPE.size(); i++) {
				m += snprintf(dBoS + m, stringSize - m, "%9.9f,", allPE[i]);
			///
			dataStream3.write(dBoS, m);
			dataStream3.clear();
			dataStream3.close();
			
		}
		*/
		

		// else if here on an escape chart, return now to not fuck up fxnStop variable
		if (escaping) {
			return;
		}
		altSoi = -1; // nested chartTraj wasnt designed for, need to set it back to -1 when it's bubbled back
		fxnStop = true;
	}
	fxnStop = false;
}

void ArtSat::fillBuff(int mod) {
	mtxSat->lock();
	// thread change guard
	while ((threadStop == nullptr || !(threadStop->load())) && !fxnStop && dynBuff != nullptr && dynBuff->size() != 0) { // && *soi_ing == mod

		int j = (LINE_BUFF_SIZE_AS * (lineBuffSect.size() - 1)) + 1; float overflow = 0; int overflowAmt = 0;
		bool sumUnderOne = false;


		// fill step lineBuff with dynLineBuff
		float sum = dynBuff->size() / (float)((LINE_BUFF_SIZE_AS - 2) / 2);



		// handle sub orbital trajectories, DONT NEED ALL THIS LOGIC
		if (sum < 1) {
			sumUnderOne = true;
			lB_size_actual = dynBuff->size();
		}
		else if (mod == 1) {
			lB_size_actual = LINE_BUFF_SIZE_AS * lineBuffSect.size();
		}
		else
			lB_size_actual = -1;

		// fill loop, divides dynLBS evenly depending on line buff size
		for (int k = 0; k < dynBuff->size(); k++) {
			if (!sumUnderOne) {
				while (((k - overflowAmt) % (int)sum) != 0 && k < dynBuff->size()) {
					k++;
				}
				overflow += sum - (int)sum;
				if (overflow > 1) {
					k++; overflowAmt++;
					overflow = overflow - (int)overflow;
				}
			}
			if (k < dynBuff->size() && j < (LINE_BUFF_SIZE_AS * lineBuffSect.size()) - 2) {
				lineBuff[j] = { (*dynBuff)[k] };
				lineBuff[j + 1] = lineBuff[j];
				lBTime[(j) / 2] = (*dynTimes)[k];
				j += 2;
			}
		}



		if (mod == 0) {
			lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[0];
		}
		if (mod == 1) {
			lineBuff[(LINE_BUFF_SIZE_AS * (lineBuffSect.size() - 1))] =
				lineBuff[(LINE_BUFF_SIZE_AS * (lineBuffSect.size() - 1)) + 1];
			lineBuff[(LINE_BUFF_SIZE_AS * lineBuffSect.size()) - 1] =
				lineBuff[(LINE_BUFF_SIZE_AS * lineBuffSect.size()) - 2];
		}

		lBTime[(LINE_BUFF_SIZE_AS / 2) - 1] = (*dynTimes)[dynTimes->size() - 1];


		// ensure closure only when the orbits endpoints are close
		if (distanceFind(lineBuff[LINE_BUFF_SIZE_AS - 1].Pos, lineBuff[LINE_BUFF_SIZE_AS - 2].Pos) >
			(2 * distanceFind(lineBuff[LINE_BUFF_SIZE_AS - 2].Pos, lineBuff[LINE_BUFF_SIZE_AS - 3].Pos))) {
			lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[LINE_BUFF_SIZE_AS - 2];
		}
		fxnStop = true;
	}
	fxnStop = false;
	mtxSat->unlock();
}

void ArtSat::chartApproach(std::vector<Mesh*> bodies, int targetID) {

	double prelimDist = INT_MAX, dist = INT_MAX, dummy = 0;
	int closestNode = 0;


	// loop through LB time and find closest pt in orbit to planet, then march back one pt and iterate to get the exact pt
	for (int i = 0; i < (lineBuffSect.size() * LINE_BUFF_SIZE_AS) - 2;) { // only searches x * lbsz pts
		double dummyDist = distanceFind(lineBuff[i].Pos, bodies[targetID]->getPV(lBTime[i/2], true).Pos);
		if (dummyDist < prelimDist) {
			prelimDist = dummyDist;
			closestNode = i;
		}
		i += 2;
	}

	dist = INT_MAX;
	pvUnit pv = lineBuff[closestNode - 2], pvb4 = pv;
	invStateChange(&(pv.Pos), &(pv.Vel));
	double time = lBTime[(closestNode - 2) / 2];
	double timeStep = 10.0;

	while (1) {
		updatePV(&pv, bodies, soiIdx, time, timeStep, dummy);
		double dummyDist = distanceFind(pv.Pos, bodies[targetID]->getPV(time + timeStep, false).Pos);
		if (dummyDist < dist) {
			dist = dummyDist;
			break;
		}
		dist = dummyDist;
		time += timeStep;
		pvb4 = pv;
		if (time > lBTime[(closestNode - 2) / 2]) { // lazy error fixing
			dist = prelimDist;
			time = lBTime[(closestNode - 2) / 2];
			break;
		}

	}	

	mk1 = new Marker(0, 1, (glm::vec3)pvb4.Pos, *(bodies[soiIdx]->Pos));
	if (bodies[targetID]->isMoon)
		mk2 = new Marker(0, 1, (glm::vec3)(bodies[targetID]->getPV(time, true).Pos), *(bodies[targetID]->gravSource->Pos)); // scheme changes for planet targets
	else
		mk2 = new Marker(0, 1, (glm::vec3)(bodies[targetID]->getPV(time, true).Pos), glm::vec3(0,0,0));
	targStat->closeAppr = dist;
	closeApproachTime = time;
}

void ArtSat::refreshTraj(std::vector<Mesh*> bodies, double dt) {
	chartTraj(*state, bodies, dt, -1);
	if (targStat != nullptr)
		chartApproach(bodies, targStat->targetIdx);
}

// integrate two traj's and add maneuver to bridge the gap
void ArtSat::solveManeuver(std::vector<Mesh*> bodies, char* name, pvUnit pv1, pvUnit pv2, double t1, double t2) {
	std::vector<pvUnit> pvAL, pvBL;
	pvUnit pvA = pv1, pvB = pv2;
	double dummy, timeStep = 3;

	// compile trajectories
 	for (int i = 0; i < t2 - t1; ) { 
		pvAL.push_back(pvA);
		pvBL.push_back(pvB);
		updatePV(&pvA, bodies, 3, t1 + i, timeStep, dummy);
		updatePV(&pvB, bodies, 3, t2 - i, -timeStep, dummy);
		i += timeStep; 
	}
	// flip backprop, could just reverse later i idx and save the time
	for (int i = 0; i < pvBL.size() / 2; i++) { 
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
	int soi = 3;
	soiDetector(bodies, pvBL[i].Pos, t1 + (timeStep * i), &soi, -1);
	maneuvers.push_back({ pvAL[i], pvBL[i],  t1 + (timeStep * i), saveName, "?", soi});
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

	pvUnit soi = bodies[soiIdx]->getPV(dt, false);
	if (bodies[soiIdx]->isMoon) {
		soi = soi + bodies[soiIdx]->gravSource->getPV(dt, false);
	}
	pvUnit pvToSun = pv + soi;

	std::vector<double> str; // hold influence of each body

	for (int i = 0; i < bodies.size(); i++) {
		pvUnit pvToBody = pvToSun;
		if (i != 0) {
			pvToBody = pvToBody - bodies[i]->getPV(dt, false);
		}
		if (bodies[i]->isMoon) {
			// if planet not within 30 units distance do not include moons in grav calc
			pvUnit pvToSunSC = pvToSun;
			stateChange(&pvToSunSC.Pos, &pvToSunSC.Vel);
			if (double planetCheck = abs(vecMag(*(bodies[i]->gravSource->Pos) - (glm::vec3)pvToSunSC.Pos)) > 30) { 
				continue;
			}
			pvUnit moonAmt = bodies[i]->gravSource->getPV(dt, false) + bodies[i]->getPV(dt, false);
			pvToBody = pvToSun - moonAmt;
		}
		double dist = abs(vecMag(pvToBody.Pos));
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

pvUnit RK4(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep) {
	// coupled runge kutta 4th order, dt is the step size h
	
	/*
	double flipper = 1; // handle reverse time
	if (timeStep < 0) {
		//flipper = -1;
		//dt = -dt;
	}
	
	// RK3 4th order
	pvUnit dPV = pv;
	glm::dvec3 k1 = accel_comp(dPV, bodies, soiIdx, dt);
	dPV.Pos = pv.Pos + (timeStep / 2.0) * dPV.Vel;
	dPV.Vel = pv.Vel + (timeStep / 2.0) * k1;
	glm::dvec3 k2 = accel_comp(dPV, bodies, soiIdx, dt + (timeStep / 2.0));
	dPV.Pos = pv.Pos + dt * dPV.Vel;
	dPV.Vel = pv.Vel + -(timeStep * k1) + (2 * timeStep * k2);
	glm::dvec3 k3 = accel_comp(dPV, bodies, soiIdx, dt + timeStep);

	dPV.Vel = pv.Vel + timeStep * ((1.0 / 6.0) * k1 + (2.0 / 3.0) * k2 + (1.0 / 6.0) * k3);
	dPV.Pos = pv.Pos + timeStep * dPV.Vel;
	return dPV;
	*/


	
	// RK4 4th order
	glm::dvec3 k1 = pv.Vel;
	glm::dvec3 l1 = accel_comp({pv.Pos, k1}, bodies, soiIdx, dt); // instantaneous acceleration, compare with bodies

	glm::dvec3 r2 = pv.Pos + ((timeStep / 2) * k1);
	glm::dvec3 k2 = pv.Vel + ((timeStep / 2) * l1); // also v2
	glm::dvec3 l2 = accel_comp({ r2, k2 }, bodies, soiIdx, dt);

	glm::dvec3 r3 = pv.Pos + ((timeStep / 2) * k2);
	glm::dvec3 k3 = pv.Vel + ((timeStep / 2) * l2); // also v3
	glm::dvec3 l3 = accel_comp({ r3, k3 }, bodies, soiIdx, dt);

	glm::dvec3 r4 = pv.Pos + (timeStep * k3);
	glm::dvec3 k4 = pv.Vel + (timeStep * l3); // also v4
	glm::dvec3 l4 = accel_comp({ r4, k4 }, bodies, soiIdx, dt);

	return  pv + pvUnit{ (timeStep / 6) * (k1 + (k2 * 2.0) + (2.0 * k3) + k4) , (timeStep / 6) * (l1 + (l2 * 2.0) + (2.0 * l3) + l4) };
	}

pvUnit composed_RK4(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep) {
	// symmetric 2-step decrease error for near time-reversibility
	pvUnit res = pv;
	res = RK4(res, bodies, soiIdx, dt, timeStep / 2);
	res = RK4(res, bodies, soiIdx, dt + (timeStep / 2), timeStep / 2);
	return res;
}

pvUnit leapfrog(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep) {
	// second order leapfrog algorithm (velocity-verlet)
	// requires smoothing algorithm
	pvUnit res = pv;

	glm::dvec3 halfVel = pv.Vel + (accel_comp(pv, bodies, soiIdx, dt) * (timeStep / 2));

	res.Pos = pv.Pos + (halfVel * timeStep);
	res.Vel = halfVel + (accel_comp({ res.Pos, halfVel }, bodies, soiIdx, dt) * (timeStep / 2));

	return res;
}

pvUnit yoshida(pvUnit pv, std::vector<Mesh*> bodies, int soiIdx, double dt, double timeStep) {
	// 4th order Yoshida integrator (modified leapfrog)
	// does not require smoothing
	glm::dvec3 pUpd[4], vUpd[4];
	pUpd[0] = pv.Pos; vUpd[0] = pv.Vel;
	// establish coeffcients
	double c[4], d[3];
	c[0] = yw1 / 2;
	c[1] = (yw0 + yw1) / 2;
	c[2] = c[1];
	c[3] = c[0];

	d[0] = yw1;
	d[1] = yw0;
	d[2] = d[0];
	// integrate
	for (int i = 1; i < 4; i++) {
		pUpd[i] = pUpd[i - 1] + (c[i - 1] * vUpd[i - 1] * timeStep);
		vUpd[i] = vUpd[i - 1] + (d[i - 1] * accel_comp({ pUpd[i], vUpd[i - 1] },
			bodies, soiIdx, dt + timeStep ) * timeStep);
	}

	pvUnit res;
	res.Pos = pUpd[3] + (c[3] * vUpd[3] * timeStep);
	res.Vel = vUpd[3];	

	return res;
}

glm::dvec3 onePN(pvUnit pv, double M) {
	// 1PN approximation
	double c2 = 299792458; c2 = c2 * c2; // m/s
	double G = 6.6743 * glm::pow(10, -11); // N * m^2 * kg^-2
	pv.Pos *= 1000; pv.Vel *= 1000; // values in meters
	double magR = vecMag(pv.Pos);
	double magV = vecMag(pv.Vel);

	double gmOverR3 = -(G * M) / (magR * magR * magR); 
	glm::dvec3 accNewt = gmOverR3 * pv.Pos;

	glm::dvec3 accRel = accNewt * (-(magV * magV) + ((double)(4 * G * M) / magR));
	accRel /= c2;

	glm::dvec3 accCross = -gmOverR3 * (4 / c2) * (glm::dot(pv.Pos, pv.Vel) * pv.Vel);

	glm::dvec3 accSelf = -gmOverR3 * (3 / c2) * (magV * magV) * pv.Pos;

	accNewt += accRel + accCross + accSelf;
	accNewt /= 1000; // back to km
	return accNewt; 
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
	*pv = RK4(*pv, bodies, soiIndex, dt, (double)timeStep);
}
// if specifyCheck is -1, loops all bodies + returns true on change - else returns true if in specific soi, false if not
bool soiDetector(std::vector<Mesh*> bodies, glm::dvec3 satPos, double dt, int* soiIdx, int specifyCheck) {
	
	bool found = false;
	int newSoiIdx = -1;
	
	// find satPos rel to sun
	pvUnit soi = bodies[*soiIdx]->getPV(dt, false);
	if (bodies[*soiIdx]->isMoon) {
		soi = soi + bodies[*soiIdx]->gravSource->getPV(dt, false);
	}
	glm::dvec3 satPos2Sun = satPos + soi.Pos, satPos2Body = satPos2Sun;

	// if specifyCheck, constrict loop to that body
	int loopBound1 = 1, loopBound2 = bodies.size();
	if (specifyCheck > 0) {
		loopBound1 = specifyCheck;
		loopBound2 = specifyCheck + 1;
	}
	// loop all bodies, moons first for simplicity
	for (int i = loopBound2 - 1; i >= loopBound1; i--) {
		// get satPos rel to body
		satPos2Body = satPos2Sun - bodies[i]->getPV(dt, false).Pos;

		if (bodies[i]->isMoon) {
			pvUnit moonAmt = bodies[i]->gravSource->getPV(dt, false) + bodies[i]->getPV(dt, false);
			satPos2Body = satPos2Sun - moonAmt.Pos;
		}
		if (abs(vecMag(satPos2Body)) < bodies[i]->soiRadius) {
			newSoiIdx = i;
			break;
		}
	}
	if (newSoiIdx == -1) newSoiIdx = 0;

	if (specifyCheck != -1) {
		if (newSoiIdx == specifyCheck)
			return true;
		else
			return false;
	}

	if (newSoiIdx == *soiIdx) 
		return false;
	*soiIdx = newSoiIdx;
	return true;
}

// returns pv wrt relativeBody assuming it's input from currBody
pvUnit translatePV(pvUnit pv, std::vector<Mesh*> bodies, double dt, int currBody, int relativeBody) {
	pvUnit soi = bodies[currBody]->getPV(dt, false);
	if (bodies[currBody]->isMoon) {
		soi = soi + bodies[currBody]->gravSource->getPV(dt, false);
	}
	pvUnit satPos2Sun = pv + soi, satPos2Body = satPos2Sun;
	if (relativeBody != 0) {
		satPos2Body = satPos2Sun - bodies[relativeBody]->getPV(dt, false);
	}
	if (bodies[relativeBody]->isMoon) {
		pvUnit moonAmt = bodies[relativeBody]->gravSource->getPV(dt, false) + bodies[relativeBody]->getPV(dt, false);
		satPos2Body = satPos2Sun - moonAmt;
	}
	return satPos2Body;
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

// calculates adapting time factor for efficient orbit drawing
double timeFactorCalc(pvUnit* pv, std::vector<Mesh*> bodies, int soiID) {

	/*
	update time step based on position
	constants are all tweaked parameters - the divisor in the exponent of expFactor is the most impactful

	lower ratio means 
	*/
	double distToSun = vecMag(pv->Pos);
	double factor = 1;
	double mRatio = bodies[soiID]->mass / bodies[3]->mass;
	double mFactor = pow(10.0, -log10(mRatio) * (3.0 / 7.0));
	double posFactor = (vecMag(pv->Pos) - bodies[soiID]->realRadius) * mFactor / 1000;

	//if (mRatio > 10)
		//posFactor *= mFactor * 1000;

	double expFactor = exp(posFactor / 485.0) + 1;

	float factorThresh = 1.5;
	if (posFactor < factorThresh)
		factor = posFactor;
	else if (posFactor / expFactor < factorThresh && posFactor < 1000)
		factor = factorThresh;
	else
		factor = posFactor;

	if (posFactor / expFactor < -FLT_MAX) // lazy fix
		factor = 0;
		

	// calculate escape velocity
	//double G = 6.6743 * glm::pow(10, -11);
	//double e_vel = sqrt((2 * G * bodies[soiID]->mass) / (bodies[soiID]->realRadius * 1000));
	//double factor = 1 - (abs(vecMag(pv->Vel)) / e_vel);

	return factor;
}

// determine when orbits should close based on angle
bool angleCalc(pvUnit* pvCurr, std::vector<double> &angleList, int iterations, double &firstAngle, double &lastAngle, int &lastQuad, int &quadChange, int &angleAxis, bool &lostAxis, bool &cw) {
	// a & b check for turning points
	double aAngle = getAngle(pvCurr->Pos, pvCurr->Vel) - (glm::pi<double>() / 2);
	int axisFind = 0;
	for (int i = 0; i < 3; i++) {
		if (abs(pvCurr->Pos[i]) <= abs(pvCurr->Pos[axisFind]))
			axisFind = i;
	}
	if (angleAxis == -1) {
		angleAxis = axisFind;
	}
	else if (angleAxis != axisFind)
		lostAxis = true;


	std::vector<double> pt;
	for (int i = 0; i < 3; i++) {
		if (i != angleAxis)
			pt.push_back(pvCurr->Pos[i]);
	}

	double currAngle;
	int currQuad;

	currAngle = atan2(pt[1], pt[0]);
	if (currAngle < 0)
		currAngle += 2 * glm::pi<double>();

	if (pt[0] > 0 && pt[1] < 0) {
		currQuad = 1;
	}
	else if (pt[0] < 0 && pt[1] < 0) {
		currQuad = 2;
	}
	else
		currQuad = 3;


	if (iterations == 1) {
		firstAngle = currAngle;
	}
	else if (iterations == 2) { // det spin dir
		if (currAngle > firstAngle)
			cw = true;
		else
			cw = false;
	}
	else if (lastQuad != currQuad) {
		quadChange++;
	}
	else if (quadChange >= 3) {
		lastAngle = currAngle;
		if ((cw && lastAngle > firstAngle) || (!cw && lastAngle < firstAngle))
			return true;
	}

	lastAngle = currAngle;
	lastQuad = currQuad;
	angleList.push_back(currAngle);
	return false;
}

void buffAppend(pvUnit* &lB, vertex_t* &relLB, double* &lBT, std::vector<std::pair<int, int>> &lineBuffSect) { // ASSUMES BUFFERS ARE FIXED SIZE, NEED TO ADD SECT SECONDS

	pvUnit* dummyBuff = new pvUnit[LINE_BUFF_SIZE_AS * lineBuffSect.size()];
	std::copy(lB, lB + (LINE_BUFF_SIZE_AS * lineBuffSect.size()), dummyBuff);

	delete[] lB;
	lB = new pvUnit[LINE_BUFF_SIZE_AS * (lineBuffSect.size() + 1)];
	std::copy(dummyBuff, dummyBuff + (LINE_BUFF_SIZE_AS * (lineBuffSect.size())), lB);
	delete[] dummyBuff;

	
	vertex_t* dummyVertex = new vertex_t[LINE_BUFF_SIZE_AS * lineBuffSect.size()];
	std::copy(relLB, relLB + (LINE_BUFF_SIZE_AS * lineBuffSect.size()), dummyVertex);

	delete[] relLB;
	relLB = new vertex_t[LINE_BUFF_SIZE_AS * (lineBuffSect.size() + 1)];
	std::copy(dummyVertex, dummyVertex + (LINE_BUFF_SIZE_AS * (lineBuffSect.size())), relLB);
	delete[] dummyVertex;

	
	double* dummyTime = new double[LINE_BUFF_SIZE_AS * lineBuffSect.size() / 2];
	std::copy(lBT, lBT + (LINE_BUFF_SIZE_AS * lineBuffSect.size() / 2), dummyTime);

	delete[] lBT;
	lBT = new double[LINE_BUFF_SIZE_AS * (lineBuffSect.size() + 1) / 2];
	std::copy(dummyTime, dummyTime + (LINE_BUFF_SIZE_AS * lineBuffSect.size() / 2), lBT);
	delete[] dummyTime;
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

