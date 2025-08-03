#include "ArtSat.h"

pvUnit RK4(pvUnit pv, double M, float dt);
glm::vec3 onePN(pvUnit pv, double M);
float vecMag(glm::vec3 vec);
//pvUnit wrt(pvUnit pv, pvUnit bodyPV);
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiID, double dt, float timeStep);

std::vector <double> createGaussianKernel(int size, int stdDev);
int gSmooth(std::vector<double> *kernel, std::vector<vertex_t> *noisy, std::vector<vertex_t> *smooth);
float getAngle(glm::vec3 v1, glm::vec3 v2);

ArtSat::ArtSat() {
	// init list of all art sats and their maneuvers
	//for (int i = 0; i < LINE_BUFF_SIZE_AS - 1; i++)
	//	lineBuff[i] = *(new vertex_t{});
	// could begin with circular orbit at some default height based on radius of soiID

	// init shader device, will need to get uniform for camera info to render
	ArtSat::pathDevice = geom_shdr_lines_init_device();
}

// do not pass soiID, needs index of soi in bodies
int ArtSat::ArtSatPlan(pvUnit pv, double dt, int soiIndex, std::vector <Mesh*> bodies) {
	// on the fly position and velocity modification initialization with visual updates

	// only runs program if lineBuff if a new input is given
	if (prevPV == nullptr || *prevPV != pv) { // no memory held here sometimes? (*prevPV)
		if (prevPV == nullptr)
			prevPV = new pvUnit(pv);
		else
			*prevPV = pv;


		float min = 60.0f;


		pvUnit* pvCurr = new pvUnit(pv);
		pvUnit* pvSC = new pvUnit(pv);
		stateChange(&(pvSC->Pos), &(pvSC->Vel));
		lineBuff[0] = vertex_t{ {pvSC->Pos + *bodies[soiIndex]->Pos, lineWidth}, lineColor }; //***** lineBuff positions should be state changed
		double itTime = dt;

		int testDist = 0;
		int iterations = 0;
		bool badFlag = true;

		pvUnit v1 = pv, v2 = pv;
		float angleWide = INT_MAX; double vertexTime; float angleSum = 0;
		float returnDist = 0; float dummyDist;
		int ratio = 4;

		/*
		updatePV(pvCurr, bodies, soiIndex, itTime, 3 * min);
		itTime += (3 * min);
		if (v2 == pv) {
			v2 = *pvCurr;
			continue;
		}
		if (v1 == pv) {
			v1 = v2;
			v1 = *pvCurr;
			continue;
		}
		if (angleSum > 180) {
			break;
		}


		iterations++;
		glm::vec3 u = v2.Pos - v1.Pos;
		glm::vec3 v = pvCurr->Pos - v2.Pos;
		float angle = acosf(glm::dot(u, v) / (vecMag(u) * vecMag(v)));
		angleSum += angle;

		if (angle < angleWide) {
			angleWide = angle;
			vertexTime = dt;
		}
	*/
	/* how to make chart step most robust??

	chart only one orbit with outocmes:
	closed orbit, open orbit, soi change, suborbital

	find apoapsis by judging distances of newly iterated points from body center
	find periapsis the same way
	judge distance to original point (pv) and detect closest approach, choosing to close or open depending on distance
	to pv

	should be able to update open orbits on sim time, LIFO-ing the buffer circularly

	*********
	from a given pv need to know when to start looking for closure - will find ap and peri along the way & save it #'s
	*********

	at a given pv there is only one other spot in an orbit w/ same distance to planet
	once that point is passed, the next time the point gets close to that same distance is
	at endpt ---------- unless pv is apo / peri

	look for first vertex (apo / peri) - dist to body
		look for pv similar - dist to body -> confirms first vertex, need to know which
			look for second vertex* - dist to body
				look for closure - dist to body (choose open/close from dist to pv) -> confirms second vertex

	afraid circular orbits with jagged predictions may disrupt this approach
	depends on how jagged - could also smooth out points at end with a noise filter

	velocity vector is input, angle compared to body-earth line tells if apo/peri is coming first unless
	90 degrees, then must judge distnace by next 90 degree point

	need fxn to tell angle of vel
	*/

	// scan step w/ dynamic gaussian smoothing
		std::vector <vertex_t> dynLBN, dynLBS;
		double pvDist = 0;
		bool atTP = false, close = false, twin = false;
		if (abs(getAngle(pv.Pos, pv.Vel) - (glm::pi<float>() / 2)) < 0.1) atTP = true;
		
		// generate Gaussian Kernel here
		std::vector <double> gKernel = createGaussianKernel(101, 15);
		while (!close) {

			updatePV(pvCurr, bodies, soiIndex, itTime, min / ratio);
			itTime += min / ratio;

			*pvSC = *pvCurr;
			stateChange(&(pvSC->Pos), &(pvSC->Vel));
			dynLBN.push_back(vertex_t{ {pvSC->Pos + *bodies[soiIndex]->Pos, lineWidth}, lineColor });
			
			if (pvDist == 0) pvDist = vecMag(pvSC->Pos);
			
			// smoothing fxn + offset correct
			gSmooth(&gKernel, &dynLBN, &dynLBS); // first pt of dynLBS is 0

			// stop if last smooth entry changes soi or connects
			if (atTP && dynLBS.size() > 1) { // if first pt is at turning point itTime - ((min / ratio) * (gKernel.size() + 1) / 2)
				glm::vec3 thingy2 = bodies[soiIndex]->getPV(dt, true).Pos;
				// thingy2 off because getPV was set to "10" instead of "0", it's supposed soiID
				double thingy = distanceFind(dynLBS[dynLBS.size() - 1].pos, thingy2);
				if (thingy < pvDist) {
					twin = true;
				}
				if (thingy > pvDist && twin == true) close = true;
			}
			else if (!atTP && dynLBS.size() > 1) { // if first pt has a twin
				if (!twin && distanceFind(dynLBS[dynLBS.size() - 1].pos, bodies[soiIndex]->getPV(itTime, true).Pos) < pvDist) {
					twin = true;
				}
				else if (twin && distanceFind(dynLBS[dynLBS.size() - 1].pos, bodies[soiIndex]->getPV(itTime, true).Pos) > pvDist) {
					close = true;
				}
			}

		}

		std::fstream dataStream("testDataTwo.txt", std::fstream::in | std::fstream::out | std::fstream::trunc); // can read and write
		if (!dataStream) {
			std::cout << "File failed to open / create\n";
			dataStream.clear();
			return 0;
		}
		const int stringSize = 100000;
		char dBoS[stringSize];
		int m = 0;
		for (int i = 0; i < dynLBS.size(); i++) {
			m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBS[i].pos.x);
			m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBS[i].pos.y);
			m += snprintf(dBoS + m, stringSize - m, "%9.9f,", dynLBS[i].pos.z);

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

			
		// fill step lineBuff with dynLineBuff
		float sum = dynLBS.size() / (float)((LINE_BUFF_SIZE_AS - 2) / 2);
		std::cout << "dynLineBuff size: " << dynLBS.size() << "  , sum: " << sum << '\n';
		int j = 1; float overflow = 0; int overflowAmt = 0;
		bool sumUnderOne = false; bool overflowNow = false;

		if (sum < 1) sumUnderOne = true;

		for (int k = 0; k < dynLBS.size();) {
			overflowNow = false;
			while (!sumUnderOne && ((k + overflowAmt) % (int)sum) != 0 && k < dynLBS.size()) {
				k++;
			}
			overflow += sum - (int)sum;
			if (overflow > 1) {
				k++; overflowAmt++; overflowNow = true;
				overflow = overflow - (int)overflow;
			}
			if (k < dynLBS.size() && (overflowNow || !sumUnderOne) && j < LINE_BUFF_SIZE_AS - 1) { // shouldnt need last check
				lineBuff[j] = dynLBS[k];
				lineBuff[j + 1] = lineBuff[j];
				j += 2;
			}
			k++;
		}

		lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[0];
					
		// should obtain orbitalPeriod, apo, peri, time to each as well
		//delete pvCurr;
		//delete pvSC;
		return 1;
	}
}

void ArtSat::ArtSatSave() {
	// lock in and add plan to set of all art sats



}

void ArtSat::ArtSatManeuver() {
	// modify position and velocity (instantaneously or over some dt burn time)
	// add maneuver details (at/over met times) to list of spacecraft history

}

void ArtSat::ArtSatUpdState() {
	// divide time step up here, then summation all bodies calling RK4 fxn



}

void ArtSat::ArtSatRender(Camera* camera) {
	// render lineBuff
	uniform_data_t uni;
	glm::mat4 mvp = camera->cameraMatrix * glm::mat4(1.0); // mult 4x4 glm - non zero, should check shader
	uni.mvp = &mvp[0][0];
	glm::vec4 vpt = glm::vec4(0, 0, (float)camera->width, (float)camera->height);
	uni.viewport = &vpt.z;
	glm::vec2 aa_radii = glm::vec2(2.0f, 2.0f);
	uni.aa_radius = &aa_radii.x;

	geom_shdr_lines_update(&pathDevice, &lineBuff,
		LINE_BUFF_SIZE_AS, sizeof(vertex_t), &uni);
	geom_shdr_lines_render(&pathDevice, LINE_BUFF_SIZE_AS);

}



void ArtSat::deleteArtSat() {
	// delete pvunit, lineBuff, obj Mesh, and locked pos and vel init

}

std::vector <double> createGaussianKernel(int size, int stdDev) {
	// size must be odd
	
	if (size % 2 == 0) {
		size += 1;
	}
	std::vector <double> res(size, 0.0f);
	int mdpt = (size - 1) / 2;
	float sum = 0;
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

pvUnit RK4(pvUnit pv, double M, float dt) {
	// coupled runge kutta 4th order, dt is the step size h
	glm::vec3 k1 = pv.Vel;
	glm::vec3 l1 = onePN(pv, M); // instantaneous acceleration, compare with bodies

	glm::vec3 r2 = pv.Pos + ((dt / 2) * k1);
	glm::vec3 k2 = pv.Vel + ((dt / 2) * l1); // also v2
	glm::vec3 l2 = onePN(pvUnit{ r2, k2 }, M);

	glm::vec3 r3 = pv.Pos + (k2 * (dt / 2));
	glm::vec3 k3 = pv.Vel + ((dt / 2) * l2); // also v3
	glm::vec3 l3 = onePN(pvUnit{ r3, k3 }, M);

	glm::vec3 r4 = pv.Pos + (dt * k3);
	glm::vec3 k4 = pv.Vel + (dt * l3); // also v4
	glm::vec3 l4 = onePN(pvUnit{ r4, k4 }, M);

	return pvUnit{ ((dt / 6) * k1 + (k2 * 2.0f) + (2.0f * k3) + k4) , ((dt / 6) * l1 + (l2 * 2.0f) + (2.0f * l3) + l4) };
}

glm::vec3 onePN(pvUnit pv, double M) {
	// 1PN approximation
	float c2 = 299792458; c2 = c2 * c2; // m/s
	float G = 6.6743 * glm::pow(10, -11); // N * m^2 * kg^-2
	pv.Pos *= 1000; pv.Vel *= 1000; // values in meters
	float magR = vecMag(pv.Pos);
	float magV = vecMag(pv.Vel);

	float gmOverR3 = -(G * M) / (magR * magR * magR);
	glm::vec3 accNewt = gmOverR3 * pv.Pos;

	glm::vec3 accRel = accNewt * ((magV * magV) - ((float)(4 * G * M) / magR));
	accRel /= c2;

	glm::vec3 accCross = -gmOverR3 * (4 / c2) * (glm::dot(pv.Pos, pv.Vel) * pv.Vel);

	glm::vec3 accSelf = -gmOverR3 * (3 / c2) * (magV * magV) * pv.Pos;

	accNewt += accRel + accCross + accSelf;
	accNewt /= 1000; // back to km
	return accNewt; // supposed to be small
}

float vecMag(glm::vec3 vec) {
	float mag = (vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z);
	return sqrt(mag);
}

float getAngle(glm::vec3 v1, glm::vec3 v2) {
	return acosf(glm::dot(v1, v2) / (vecMag(v1) * vecMag(v2)));
}

// is this needed? just the adding / subtracting below should do it (updatePV)
//pvUnit wrt(pvUnit pv, pvUnit bodyPV) {
	// change pv (wrt to soiID) to the pv from frame of body given it's pv


//}


// update pv with 1PN approximation at specific time dt (UTC) pv untis should be in km (NOT state changed)
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiIndex, double dt, float timeStep) {
	pvUnit* pvDummy = new pvUnit{ glm::vec3{0,0,0}, glm::vec3{0,0,0} };
	pvUnit* pvSun = new pvUnit(*pv);
	float G = 6.6743 * glm::pow(10, -11); // N * m^2 * kg^-2
	float mainPot = (G * bodies[soiIndex]->mass) / pow(vecMag(pv->Pos), 2);
	if (bodies[soiIndex]->baryID != 10) {
		*pvSun = *pvSun + bodies[soiIndex]->getPV(dt, false);
	}

	//*pvDummy = *pvDummy + RK4((*pvSun), bodies[0]->mass, timeStep);
	for (int j = 0; j < bodies.size(); j++) {
		// needs fxn p&v from pov of each body, need function
		pvUnit change = RK4((*pvSun - bodies[j]->getPV(dt, false)), bodies[j]->mass, timeStep);
		float altPot = (G * bodies[j]->mass) / pow(vecMag((*pvSun - bodies[j]->getPV(dt, false)).Pos), 2);
		change = change * (altPot / mainPot);
		// does getPV work for sun
		*pvDummy = *pvDummy + change;

	}
	
	*pvSun = *pvSun + *pvDummy;
	*pv = *pvSun - bodies[soiIndex]->getPV(dt, false);
	delete pvDummy;
	delete pvSun;
}

int gSmooth(std::vector<double>* kernel, std::vector<vertex_t>* noisy, std::vector<vertex_t>* smooth) {
	// init
	int k = (kernel->size() - 1) / 2;
	double sum = 0;
	glm::vec3 dummy( 0.0f, 0.0f, 0.0f );
	std::vector<double> dKernel = *kernel;

	if (noisy->size() < k + 1) return 1;
	if (noisy->size() < kernel->size()) {
		// truncated kernel
		std::vector<double> tKernel(kernel->begin() + k + k - noisy->size(),
			kernel->begin() + noisy->size()); // make
		for (auto pt : tKernel) sum += pt;
		dKernel = tKernel;
	}
	
	// utilize kernel
	for (int i = 0; i < dKernel.size(); i++) {
		if (sum != 0) dKernel[i] /= sum;
		dummy += noisy->at(noisy->size() - dKernel.size() + i).pos * (float)dKernel[i];
	}
	smooth->push_back({ dummy, (*noisy)[0].width, (*noisy)[0].col });
	return 0; 
}