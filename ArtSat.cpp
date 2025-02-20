#include "ArtSat.h"

pvUnit RK4(pvUnit pv, double M, double dt);
pvUnit leapfrog(pvUnit pv, double M, double dt);
glm::dvec3 onePN(pvUnit pv, double M);
double vecMag(glm::dvec3 vec);
void updatePV(pvUnit* pv, std::vector<Mesh*> bodies, int soiID, double dt, float timeStep, double &E0);

std::vector <double> createGaussianKernel(int size, int stdDev);
int gSmooth(std::vector<double> *kernel, std::vector<glm::dvec3> *noisy, std::vector<glm::dvec3> *smooth);
float getAngle(glm::vec3 v1, glm::vec3 v2);

double totalEnergy(pvUnit pv, double M, double* KE, double* PE);

ArtSat::ArtSat() {
	// init list of all art sats and their maneuvers
	//for (int i = 0; i < LINE_BUFF_SIZE_AS - 1; i++)
	//	lineBuff[i] = *(new vertex_t{});
	// could begin with circular orbit at some default height based on radius of soiID

	// init shader device, will need to get uniform for camera info to render
	ArtSat::pathDevice = geom_shdr_lines_init_device();
	// init box 
	sat = new Object;
	sat->Box(.0005, 0, 1, 0.2);
	
	stateTime = 0;

	VAO.Bind();
	VBO VBO(sat->vertices);
	EBO EBO(sat->indices);

	VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
	VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
	VAO.LinkAttrib(VBO, 2, 3, GL_FLOAT, sizeof(Vertex), (void*)(6 * sizeof(float)));
	VAO.LinkAttrib(VBO, 3, 2, GL_FLOAT, sizeof(Vertex), (void*)(9 * sizeof(float)));
	VAO.Unbind();
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
		lineBuff[0] = vertex_t{ {pvSC->Pos, lineWidth}, lineColor };


		// add pos as poi to carry
		*ArtSat::state = pv;
		*ArtSat::stateButChanged = *pvSC;
		ArtSat::stateTime = dt;

		stat->initTime = dt;

		// chart trajectory
		chartTraj(pv, bodies, dt);
	}
	return 1;
}






void ArtSat::ArtSatManeuver(glm::vec3 deltaV, std::vector <Mesh*> bodies, double dt) {
	// modify velocity (instantaneously or over some dt burn time)
	pvUnit newPV = *state;
	newPV.Vel += deltaV;
	maneuvers->push_back({ newPV, dt });
	chartTraj(newPV, bodies, dt);
}

void ArtSat::ArtSatUpdState(std::vector <Mesh*> bodies, double dt) {

	// handle state update
	int counter = dt - stateTime;
	double lastTime = stateTime, dummy = 0;
	while (counter > 0) {
		updatePV(state, bodies, soiIdx, lastTime, 1.0f, dummy);
		counter--; lastTime++;
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
	}
}


void ArtSat::ArtSatRender(Camera* camera, Mesh lightSource) {
	// shader
	setShader(lightSource);
	// draw box
	sp.Activate();
	VAO.Bind();

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
	delete maneuvers;
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
		factor = exp(-((velocity - 9) / 2)); // could try lV vs. V - make factor large when deltaV is large and vice versa


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

			if (c < bodies[soiIdx]->radius || (tp == 4 && toPV > lastToPV) || iterations > 15000) // arbitrary constant
				close = true;
		}

	}
	stat->apoapsis = (vecMag(apo.Pos) / (LENGTH_SCALE)) - bodies[soiIdx]->realRadius;
	stat->periapsis = (vecMag(peri.Pos) / (LENGTH_SCALE)) - bodies[soiIdx]->realRadius;
	stat->orbitalPeriod = itTime - dt; // in seconds

	delete T; delete U;
	*apoapsis = apo;
	*periapsis = peri;


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

	lineBuff[LINE_BUFF_SIZE_AS - 1] = lineBuff[0];

	delete pvCurr;
	delete pvSC;
}

void ArtSat::refreshTraj(std::vector<Mesh*> bodies, double dt) {
	// *************************** RESETS MET
	chartTraj(*state, bodies, dt);
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

pvUnit leapfrog(pvUnit pv, double M, double dt) {
	// second order leapfrog algorithm (velocity-verlet)
	pvUnit dummyPV = pv;
	glm::dvec3 halfVel = pv.Vel + (onePN(pv, M) * (dt / 2));
	dummyPV.Pos = pv.Pos + (halfVel * dt);
	dummyPV.Vel = halfVel + (onePN({dummyPV.Pos, halfVel}, M) * (dt / 2));
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
	pvUnit* pvChange = new pvUnit{ glm::vec3{0,0,0}, glm::vec3{0,0,0} };
	pvUnit* pvToSun = new pvUnit(*pv); // pv according to solar system bary
	pvUnit pvSoi = bodies[soiIndex]->getPV(dt, false);
	float G = 6.6743 * glm::pow(10, -11); // N * m^2 * kg^-2
	float mainPot = (G * bodies[soiIndex]->mass) / pow(vecMag(pv->Pos) * 1000, 2);
	if (bodies[soiIndex]->baryID != 10) {
		*pvToSun = *pvToSun + pvSoi;
	}


	int j = 3;
	//for (int j = 3; j < 5; j++) {
		// needs fxn p&v from pov of each body, need function

		double r = vecMag((*pvToSun - bodies[j]->getPV(dt, false)).Pos);
		if (bodies[j]->isMoon) {
			
			r = vecMag(bodies[j]->getPV(dt, false).Pos - pv->Pos);
		}

		pvUnit change = leapfrog((*pvToSun - bodies[j]->getPV(dt, false)), bodies[j]->mass, timeStep);
		float altPot = (G * bodies[j]->mass) / pow(r * 1000, 2);
		float gravMod = altPot / mainPot;
		change = change * gravMod;
		
		*pvChange = *pvChange + change; 

		// soi change indicator
		if (altPot == mainPot) {
			E0 = (int)0;
		}
		else if (altPot > mainPot) {
			E0 = (int)1;
		}
	//}


	*pvToSun = *pvToSun + *pvChange; 
	*pv = *pvToSun - pvSoi; // return to local pv
	

	delete pvChange;
	delete pvToSun;
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

