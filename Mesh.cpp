#include "Mesh.h"


int closestVertex(vertex_t* lineBuffer, int setSize, SpiceDouble* state); // finds closest vertex in buffer to state
void makeRefinedList(vertex_t* refinedList, vertex_t* lineBuffer, const int lineBufferSize, Mesh* gravSource, double et, int timeWidth, const char* soiID, int spiceID,
	 double* pbt, double* prt, int* pbtIdx, int* prtIdx, double* refListDt, const int lWidth, const glm::vec4 lColor); // generates refined List

Mesh::Mesh(const char* objName, std::vector <Vertex> vertices, std::vector <GLuint> indices, std::vector <Texture> textures, float radius, float mass, bool isLight, bool areRings,
	Shader* shaderProgram, const char* soiID, char* soiIndex, int baryIDx, int spiceIDx, double UTCtime, float orbPeriod) {
	Mesh::name = objName;
	Mesh::vertices = vertices;
	Mesh::indices = indices;
	Mesh::textures = textures;
	Mesh::areRings = areRings;
	Mesh::isLightSource = isLight;
	Mesh::soiID = soiID;
	char* saveDummy = (char*)malloc(strlen(soiIndex) + 1);
	strcpy(saveDummy, soiIndex);
	Mesh::soiIdx = saveDummy;
	Mesh::spiceID = spiceIDx;
	Mesh::baryID = baryIDx;
	Mesh::orbitalPeriod = orbPeriod; // each body has a factual orbital period in days hard coded for initialization
	Mesh::refinedList = nullptr;
	Mesh::radius = radius * LENGTH_SCALE;
	Mesh::realRadius = radius;
	Mesh::mass = mass * pow(10, 20);
	if (std::strcmp(soiID,"10")) isMoon = true;
	else isMoon = false;

	

	// set object light emission color if any
	if (isLight == true) {
		Mesh::Color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
	}
	else {
		Mesh::Color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
	}

	// init position(s)
	SpiceDouble state[6];
	SpiceDouble lt;
	double et = ((UTCtime / 86400.0) + 2440587.5); // UTC to Julian
	et = (et - 2451545.0) * 86400.0; // JD to SPICE ET	
	int ephID = spiceID;
	if (spiceID > 399 && !isMoon)  // current ephemeris requirements
		ephID = baryID;
	spkezr_c(std::to_string(ephID).c_str(), et, "ECLIPJ2000", "NONE", soiID, state, &lt);
	stateChange(state);
	Pos = new glm::vec3{ state[0], state[1], state[2] };
	oPos = new glm::vec3{ *Pos };

	// set object model position
	glm::mat4 objModel = glm::mat4(1.0f);
	objModel = glm::translate(objModel, *Pos); // Pos is nullptr here
	Mesh::Model = objModel;


	// set object shader program
	Mesh::ShaderProgram = *shaderProgram;

	// vertex attribute object to send to GPU - array of attributes or states of vertices which package VBO's to send to GPU
	VAO.Bind();

	// vertex buffer object to send to GPU - array of data on vertices to be sent to GPU
	my_VBO = new VBO(vertices);
	
	// element buffer object is an array of pointers in data in VBO allows me to arrange pointers as i please instead of copying data repeatedly
	my_EBO = new EBO(indices);

	//Link VAO to VBO
	//parameters: VBO, which vertex attribute (layout) to configure, number of vertex attributes (1 per 3-vertex), type - of vector data stored, stride - space in memory between consecutive vertex attribs, offset - of where pos data begins in buffer

	// Links VBO to Position
	VAO.LinkAttrib(*my_VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
	// Links to Normals (layout 1 for shaders, offset by 3 from the start of pos
	VAO.LinkAttrib(*my_VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
	// Links to Colors (layout 2 for shader, offset by 6 from start of pos
	VAO.LinkAttrib(*my_VBO, 2, 3, GL_FLOAT, sizeof(Vertex), (void*)(6 * sizeof(float)));
	// Links Texture (layout 3 for shaders, offset by 9 from the start of pos
	VAO.LinkAttrib(*my_VBO, 3, 2, GL_FLOAT, sizeof(Vertex), (void*)(9 * sizeof(float)));
	VAO.Unbind();
	my_VBO->Unbind();
	my_EBO->Unbind();

	// need to detect a full orbit (return to pt OR orbital period calculation)
	// return to pt would work for spacecraft imp too, worth trying
	
	// conversions
	int hour = 60 * 60;
	int day = hour * 24;
	int month = day * 30;

	// orbital line composition scripting
	if (baryID != 10 && !areRings) {
		vertex_t dummyier{}; // holds first orbital point to connect paths at end of buffer

		double tPt = 0; // current time adjusted for frame
		if (isMoon) tPt = et;
		et = (double)orbitalPeriod * (double)day; // dunno why i have to abs this
		Mesh::lBVertDt = et / LINE_BUFF_SIZE_U;

		for (int i = 0; i < LINE_BUFF_SIZE_U; i++) {
			tPt += (et) / LINE_BUFF_SIZE_U; // time at each even division of orbital period
			if (tPt >= 1577923200) { // if orbital period extends beyond data store
				lineBufferSize -= 2 * (LINE_BUFF_SIZE_U + 1 - i); // ******************************** need a way to cut out lines that bridge unfulfilled orbits
				break;
			}
			spkezr_c(std::to_string(ephID).c_str(), tPt, "ECLIPJ2000", "NONE", soiID, state, &lt);
			stateChange(state);

			vertex_t dummy{};
			dummy.pos = glm::vec3((float)state[0], (float)state[1], (float)state[2]); dummy.width = lineWidth;
			dummy.col = lineColor;
			lineBuffer[2*i] = dummy; // 2*i to offset for the repeat values to connect lines together (1,2,2,3,3,4,4,1)

			if (i == 0) {
				dummyier = dummy;
			}
			else if (i == LINE_BUFF_SIZE_U - 1) {
				lineBuffer[2*i + 1] = dummyier;
				lineBuffer[2*i - 1] = dummy;
			}
			else {
				lineBuffer[2*i - 1] = dummy;
			}
		}
		Mesh::pathDevice = geom_shdr_lines_init_device();
	}



	
	/*


	if i make it so moon orbits only get made in a certain radius and delete themselves outside of it then
	i might have to make an orbit for each at that specific time period
	(let it be known i made it work in the first half hour of me working today)

	i could use the refined method for moons, stretching the bounds out from the body and closing when the distance
	between endpts is under some length
	under timewarp the last endpoint can just update itself

	can dynamically allcoate memory for the above method, should also update all refined lists to do the same thing to save on memory
	hold a pointer to the thing in the class itself
	create it with the 'new' keyword in a function
	make sure to delete when necessary

	use orbital period and number of points to get the average time per division
	start half of the orbital period before 'now' and push the loop until it spins around

	could also do this for mercury, also somewhat volatile orbit, though it'd have to be updated unlike the other lineBuffers

	*/
}

void Mesh::setShadowShader(Shader& program, glm::mat4 lightSpaceMatrix) {
	Mesh::shadowShaderProgram = program;
	Mesh::lsMatrix = lightSpaceMatrix;
}

void Mesh::switchShader() {
	if (shadowShaderProgram.ID != NULL) {
		Shader dummy = shadowShaderProgram;
		shadowShaderProgram = ShaderProgram;
		ShaderProgram = dummy;
	}
}

void Mesh::setDepthMap(GLuint depthMapInput) {
	depthMap = depthMapInput;
}

void Mesh::emissionShader() {
	(ShaderProgram).Activate();
	// light shader position uniform
	glUniformMatrix4fv(glGetUniformLocation((ShaderProgram).ID, "model"), 1, GL_FALSE, glm::value_ptr(Model));
	// light shader color uniform (sun on sun)
	glUniform4f(glGetUniformLocation((ShaderProgram).ID, "lightColor"), Color.x, Color.y, Color.z, Color.w);

}

void Mesh::dullShader(Mesh& lightSource) {
	(ShaderProgram).Activate();
	// model shader position uniform
	glUniformMatrix4fv(glGetUniformLocation((ShaderProgram).ID, "model"), 1, GL_FALSE, glm::value_ptr(Model));

	// model shader color uniform (sun on earth) and shader uniform
	glUniform4f(glGetUniformLocation((ShaderProgram).ID, "lightColor"), lightSource.Color.x, lightSource.Color.y, lightSource.Color.z, lightSource.Color.w);
	glUniform3f(glGetUniformLocation((ShaderProgram).ID, "lightPos"), (lightSource.Pos)->x, (lightSource.Pos)->y, (lightSource.Pos)->z);

	// uniform for depth and default
	glUniformMatrix4fv(glGetUniformLocation((ShaderProgram).ID, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lsMatrix));
	// uniform for default
	glUniform1i(glGetUniformLocation((ShaderProgram).ID, "shadowMap"), 1);
}

void Mesh::Draw(Camera& camera) {
	(ShaderProgram).Activate();
	VAO.Bind();

	unsigned int numDiffuse = 0;
	unsigned int numSpecular = 0;
	unsigned int i = 0;

	// counts types of textures
	for (i; i < textures.size(); i++) {
		std::string num;
		std::string type = textures[i].type;
		if (type == "diffuse") {
			num = std::to_string(numDiffuse++);
		}
		else if (type == "specular") {
			num = std::to_string(numSpecular++);
		}
		// add texture unit - send texture uniforms to frag shaders
		textures[i].texUnit((ShaderProgram), (type + num).c_str(), i);
		// bind texture to uniform
		textures[i].Bind();
	}

	// sets depth map from shadow render to normal render if it exists
	if (depthMap != 0) {
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, depthMap);
		depthMap = 0;
	}
	// implement camera uniform usage and draw
	camera.Matrix((ShaderProgram), "camMatrix");

	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}

void Mesh::Rotate(Mesh* lightSource, double UTCtime) { 
	SpiceDouble w[3];
	SpiceInt dim;

	(*spiceMtx).lock();
	bodvcd_c(spiceID, "PM", 3, &dim, w); // this angle is equivalent to the rotation angle, no need for rotation rates to be preprogrammed
	(*spiceMtx).unlock();

	float angle = glm::radians((float)w[0] + w[1] * ((UTCtime) / 86400)) + glm::pi<float>();
	// transforms model matrix by rotation
	Model = glm::rotate(Model, angle, Up);
	currAngleRad = angle;
	// redraws the shader for modified model
	dullShader(*lightSource);
}

void Mesh::AxialTilt(GLfloat tiltDeg) {
	SpiceDouble ra[3], dec[3];
	// the first constant in each of above is sufficient for the time scales of this program
	SpiceInt dim;

	bodvcd_c(spiceID, "POLE_RA", 3, &dim, ra);
	bodvcd_c(spiceID, "POLE_DEC", 3, &dim, dec);

	SpiceDouble range = 1, rectan[3];

	radrec_c(range, ra[0], dec[0], rectan); // turns ra & dec to rectangular coords
	double poleCoords[3];
	poleCoords[0] = rectan[1]; poleCoords[1] = rectan[2]; poleCoords[2] = rectan[0];
	poleVec = glm::vec3(poleCoords[0], poleCoords[1], poleCoords[2]);
	poleVec = glm::rotateY(poleVec, glm::radians(-23.4f)); // north pole vector in correct frame


	Model = glm::rotate(Model, glm::radians(tiltDeg), poleVec);
	axisTiltDegree = tiltDeg;
}

void Mesh::Orbit(Mesh* lightSource, double UTCtime, int timeWarpIndex, glm::vec3 cameraPos) {
	
	*oPos = *Pos;
	
	// NASA SPICE position
	SpiceDouble state[6];
	SpiceDouble lt;
	double et = ((UTCtime / 86400.0) + 2440587.5); // UTC to JD
	et = (et - 2451545.0) * 86400.0; // JD to SPICE ET

	int ephID = spiceID;
	if (spiceID > 399 && !isMoon)  // current ephemeris requirements
		ephID = baryID;

	(*spiceMtx).lock();
	spkezr_c(std::to_string(ephID).c_str(), et, "ECLIPJ2000", "NONE", soiID, state, &lt);
	(*spiceMtx).unlock();

	stateChange(state);
	glm::vec3 tempPos= glm::vec3(state[0], state[1], state[2]);
	if (isMoon && gravSource->Pos != nullptr) tempPos += *(gravSource->Pos); 
	
	*Pos = tempPos;

	// if object are rings, meshes normals must be flipped when the sun crosses the ring plane (z-axis turning points)
	if (areRings) { // below assumes the starting position is at z = 0
		if (Pos->z > 0 && !sign) {
			sign = !sign;
			for (int i = 0; i < vertices.size(); i++) {
				vertices[i].normal = -vertices[i].normal;
			}
			// must bind new normals to the shaders
			VAO.Bind();
			my_VBO->Update(vertices);
			VAO.Unbind();
			my_VBO->Unbind();
		}
		else if (Pos->z < 0 && sign) {
			sign = !sign;
			for (int i = 0; i < vertices.size(); i++) {
				vertices[i].normal = -vertices[i].normal;
			}
			VAO.Bind();
			my_VBO->Update(vertices);
			// *************** ADD BACK LINK ATTRIBUTES
			VAO.Unbind();
			my_VBO->Unbind();
		}
	}

	updateModel(*gravSource);
	dullShader(*lightSource);	// ONLY TIME lightSource is used in this fxn


	// moon lineBuffer and refinedList updating (according to gravSource position)
	if (isMoon) {
		for (int i = 0; i < lineBufferSize; i++) {
			if (lastItTime != NULL) lineBuffer[i].pos -= *(gravSource->oPos); // linebuffer made from local pos ( replaced vecState w/ *(gravSource->oPos))
			lineBuffer[i].pos += *(gravSource->Pos); // make global with source Pos
		}
		if (bIdx != -1) {
			for (int i = 0; i < refinedListSize; i++) {
				refinedList[i].pos -= *(gravSource->oPos);
				refinedList[i].pos += *(gravSource->Pos);
			}
		}
	}

	// update line positions
	double week = 60 * 60 * 24 * orbitalPeriod / 24; // ref list time gap, should base this on orbital period

	if (!areRings) {
		if (distanceFind(cameraPos, *Pos) < refinedRadius && bIdx == -1) { // if close, utilize regined list
			refinedList = new vertex_t[REF_LIST_SIZE];

			(*spiceMtx).lock();
			makeRefinedList(refinedList, lineBuffer, lineBufferSize, gravSource, et, week, soiID, ephID, &bt, &rt, &bIdx, &rIdx, &refListDt, lineWidth, lineColor);
			(*spiceMtx).unlock();

			refNodeMarkerTime = UTCtime;
			lBNodeMarkerTime = UTCtime;
		}

		else if (distanceFind(cameraPos, *Pos) < refinedRadius && bIdx != -1) { // if still in camera Pos and above has already executed, modify refinedList appropriately

			double dt = UTCtime - refNodeMarkerTime; // is negative when back time traveling

			/*
			Should widen the refList over a certain dt or find solution, high time warp can overflow the program, never gets to the redo stage
			*/
			float allRefVerts = abs(dt) / (week / REF_LIST_SIZE_U);
			int numRefVerts = floor(allRefVerts);

			if (numRefVerts >= REF_LIST_SIZE_U) { //redo
				refNodeMarkerTime = UTCtime;
				lBNodeMarkerTime = UTCtime;

				(*spiceMtx).lock();
				makeRefinedList(refinedList, lineBuffer, lineBufferSize, gravSource, et, week, soiID, ephID, &bt, &rt, &bIdx, &rIdx, &refListDt, lineWidth, lineColor);
				(*spiceMtx).unlock();
			}
			else if (numRefVerts > 0) { //modify
				// handle time overflow
				Mesh::refVertsSum += allRefVerts - numRefVerts;
				numRefVerts += floor(Mesh::refVertsSum);
				Mesh::refVertsSum = Mesh::refVertsSum - floor(Mesh::refVertsSum);

				// initialize vars for making new vertices
				refNodeMarkerTime = UTCtime;
				SpiceDouble stateDt[6];
				vertex_t dummy{}, dummyier;
				dummy.width = lineWidth;
				dummy.col = lineColor;

				// prepare bi-directional refined list modifcation
				int leader;
				if (dt > 0) {
					rt += numRefVerts * refListDt;
					bt += numRefVerts * refListDt;
					leader = rt;
					flipper = 1;
				}
				else {
					bt -= numRefVerts * refListDt;
					rt -= numRefVerts * refListDt;
					leader = bt;
					flipper = -1;
				}

				refListStartIdx += 2 * numRefVerts * flipper;
				refListStartIdx = (refListStartIdx + REF_LIST_SIZE) % REF_LIST_SIZE;

				for (int i = 0; i < numRefVerts; i++) {

					(*spiceMtx).lock();
					spkezr_c(std::to_string(ephID).c_str(), leader + (week / REF_LIST_SIZE_U) * (i + 1), "ECLIPJ2000", "NONE", soiID, stateDt, &lt); // need the simTime of the last relevant point
					(*spiceMtx).unlock();

					stateChange(stateDt);
					

					if (isMoon) {
						int i = 0;
						while (i < 3) {
							stateDt[i] += (*(gravSource->Pos))[i];
							i++;
						}
					}

					dummy.pos = glm::vec3(stateDt[0], stateDt[1], stateDt[2]);
					int index = refListStartIdx - ((2 * (numRefVerts - i - 1)) + 1) * flipper;
					index = (index + REF_LIST_SIZE) % REF_LIST_SIZE;
					
					if (index > 0) refinedList[index - 1] = dummy;
					else refinedList[REF_LIST_SIZE - 1] = dummy;

					refinedList[index] = dummy;
				}

				// handle refList -> lineBuffer connection pts bIdx, rIdx 
				double dt2 = UTCtime - lBNodeMarkerTime;
				int numLBVerts = floor(abs(dt2) / Mesh::lBVertDt);
				if (numLBVerts > 0) {
					// use num to add/sub from b/r
					bIdx += 2 * numLBVerts * flipper; bIdx = (bIdx + lineBufferSize) % lineBufferSize;
					rIdx += 2 * numLBVerts * flipper; rIdx = (rIdx + lineBufferSize) % lineBufferSize;
					lBNodeMarkerTime = UTCtime;
				}
				
				int bNode, rNode, freeNode;
				bNode = bIdx - 2 * flipper; bNode = (bNode + lineBufferSize) % lineBufferSize;
				rNode = rIdx - 2 * flipper; rNode = (rNode + lineBufferSize) % lineBufferSize;

				// compare relative distances to know when to switch
				flipper > 0 ? freeNode = bIdx : freeNode = bNode;
				double distB = distanceFind(refinedList[refListStartIdx].pos, lineBuffer[freeNode].pos);
				double distB1 = distanceFind(lineBuffer[bIdx].pos, lineBuffer[bNode].pos);

				flipper > 0 ? freeNode = rNode : freeNode = rIdx;
				double distR = refListStartIdx > 0 ?
					distanceFind(refinedList[refListStartIdx - 1].pos, lineBuffer[freeNode].pos) : distanceFind(refinedList[REF_LIST_SIZE - 1].pos, lineBuffer[freeNode].pos); // when -, idx to rIdx
				double distR1 = distanceFind(lineBuffer[rIdx].pos, lineBuffer[rNode].pos);
				if (distB > distB1) { bIdx += 2 * flipper; lBNodeMarkerTime = UTCtime; }
				if (distR > distR1) { rIdx += 2 * flipper; lBNodeMarkerTime = UTCtime; }

				if (bIdx > lineBufferSize - 1 || bIdx < 1) bIdx -= flipper * lineBufferSize;
				if (rIdx > lineBufferSize - 1 || rIdx < 1) rIdx -= flipper * lineBufferSize;

			}			
		}
		else { // if camera not in range of object, abandon
			if (bIdx != -1) bIdx = -1;
			if (rIdx != -1) rIdx = -1;
			delete[] refinedList;
			refinedList = nullptr;
			
			Mesh::refVertsSum = 0;
			Mesh::refListStartIdx = 0;

		}
	}
	Mesh::lastItTime = UTCtime;
}
/*
Mesh::~Mesh() {
	name = nullptr;
	Pos = nullptr;
	oPos = nullptr;
	soiID = nullptr;
	free(soiIdx);
	soiIdx = nullptr;
	refinedList = nullptr;
	spiceMtx = nullptr;
	ShaderProgram.Delete();
	shadowShaderProgram.Delete();
	VAO.Delete();
	if (pathDevice != nullptr) {
		geom_shdr_lines_term_device((void**)&pathDevice);
		pathDevice = nullptr;
	}
	// could/should save & delete VBO/EBO
	delete my_VBO;
	my_VBO = nullptr;
	delete my_EBO;
	my_EBO = nullptr;
}
*/
pvUnit Mesh::getPV(double time, bool stateChanged) { // *** add boolean for state change version instead
	std::lock_guard<std::mutex> lock(*spiceMtx);

	SpiceDouble state[6];
	SpiceDouble lt;
	double et = ((time / 86400.0) + 2440587.5);
	et = (et - 2451545.0) * 86400.0; // JD to SPICE ET

	int ephID = spiceID;
	if (spiceID > 399) { // current ephemeris requirements
		ephID = baryID;
	}
	spkezr_c(std::to_string(ephID).c_str(), et, "ECLIPJ2000", "NONE", soiID, state, &lt);

	
	if (stateChanged) stateChange(state);
	
	return pvUnit{ glm::vec3{state[0], state[1], state[2]}, glm::vec3{state[3], state[4], state[5]} };
}


void Mesh::updateModel(Mesh& source) {
	// change model pos and rotation and axial tilt
	// orbit disappears the model
	glm::mat4 objModel = glm::mat4(1.0f);
	Model = glm::translate(objModel, *Pos);

	Model = glm::rotate(Model, glm::radians(axisTiltDegree), glm::cross(Up, poleVec));
	Model = glm::rotate(Model, currAngleRad, Up);
}

double distanceFind(std::vector<double> pt1, std::vector<double> pt2) {
	double x2 = pt1[0] - pt2[0];
	double y2 = pt1[1] - pt2[1];
	double z2 = pt1[2] - pt2[2];
	x2 *= x2; x2 += (y2 * y2) + (z2 * z2);
	return sqrt(x2);
}

double distanceFind(glm::vec3 state1, SpiceDouble * state2) {
	double x = state1[0] - state2[0];
	double y = state1[1] - state2[1];
	x *= x; x += y * y;
	y = state1[2] - state2[2];
	x += y * y;
	return sqrt(x);
}

double distanceFind(glm::vec3 state1, glm::vec3 state2) {
	double x = state1[0] - state2[0];
	double y = state1[1] - state2[1];
	x *= x; x += y * y;
	y = state1[2] - state2[2];
	x += y * y;
	return sqrt(x);
}


void stateChange(SpiceDouble* state) { 
	// scales states by chosen value
	for (int i = 0; i < 6; i++) {
		state[i] = state[i] * LENGTH_SCALE;
	}

	// places the eccliptic horizontal and changes orientation for OpenGL
	glm::vec3 posVec = glm::vec3(state[0], state[1], state[2]);
	state[0] = posVec.y; state[1] = posVec.z; state[2] = posVec.x;

	posVec = glm::vec3(state[3], state[4], state[5]);
	state[3] = posVec.y; state[4] = posVec.z; state[5] = posVec.x;
}

// no rotate
void stateChange(glm::dvec3* pos, glm::dvec3* vel) { 
	for (int i = 0; i < 3; i++) {
		(*pos)[i] = (*pos)[i] * LENGTH_SCALE;
		(*vel)[i] = (*vel)[i] * LENGTH_SCALE;
	}
	double dummy = (*pos)[0];
	(*pos)[0] = (*pos)[1]; (*pos)[1] = (*pos)[2]; (*pos)[2] = dummy;
	dummy = (*vel)[0];
	(*vel)[0] = (*vel)[1]; (*vel)[1] = (*vel)[2]; (*vel)[2] = dummy;
}

// no rotate
void invStateChange(glm::dvec3* pos, glm::dvec3* vel) {
	for (int i = 0; i < 3; i++) {
		(*pos)[i] = (*pos)[i] / LENGTH_SCALE;
		(*vel)[i] = (*vel)[i] / LENGTH_SCALE;
	}
	double dummy = (*pos)[2];
	(*pos)[2] = (*pos)[1]; (*pos)[1] = (*pos)[0];  (*pos)[0] = dummy;
	dummy = (*vel)[2];
	(*vel)[2] = (*vel)[1]; (*vel)[1] = (*vel)[0]; (*vel)[0] = dummy;
}

int closestVertex(vertex_t* lineBuffer, int setSize, SpiceDouble* state) {
	// finds closest vertex index to whatever state input

	int flipper = 1; // decides chase or escape case for later
	if (setSize < 0) {
		flipper = -flipper;
		setSize = -setSize;
	}	

	double rad = distanceFind(std::vector<double>{lineBuffer[0].pos.x,
		lineBuffer[0].pos.y, lineBuffer[0].pos.z},
		std::vector<double>{lineBuffer[1].pos.x, lineBuffer[1].pos.y,
		lineBuffer[1].pos.z}); // line length - could take this out to main fxn and use to determine if a point should reassign bIdx / rIdx

	int min = 0, max = setSize - 2;

	// binary search
	int idx = 0;
	while (min <= max) { 
		idx = (min + max) / 2;
		if (distanceFind(lineBuffer[idx].pos, state) <= rad) {
			break;
		}
		if (distanceFind(lineBuffer[min].pos, state) < distanceFind(lineBuffer[max].pos, state)) {
			max = idx - 1;
		}
		else {
			min = idx + 1;
		}
	}
	
	if (idx == 0) idx = setSize - 1;
	if (lineBuffer[idx].pos == lineBuffer[idx - 1].pos) idx -= 1; // make sure index is first chronological instance of vertex in lineBuffer

	float dt = 3600;
	glm::vec3 newState;
	for (int i = 0; i < 3; i++) {
		newState[i] = state[i] + state[3 + i] * dt;
	}
	if (distanceFind(newState, lineBuffer[idx].pos) > distanceFind(lineBuffer[idx].pos, state)) {
		if (flipper < 0) idx += 2;
	}
	else {
		if (flipper > 0) idx -= 2;
	}
	
	idx = (idx + setSize) % setSize;

	return idx;
}

void makeRefinedList(vertex_t* refinedList, vertex_t* lineBuffer, const int lineBufferSize, Mesh* gravSource, double et, int timeWidth, const char* soiID, int spiceID,
	 double* pbt, double* prt, int* pbtIdx, int* prtIdx, double* refListDt, const int lWidth, const glm::vec4 lColor) {
	double bt = et - timeWidth/2;  *pbt = bt;
	double rt = et + timeWidth/2;  *prt = rt;

	double rfDt = (rt - bt) / REF_LIST_SIZE_U; *refListDt = rfDt;


	SpiceDouble stateBt[6];
	SpiceDouble stateRt[6];
	SpiceDouble stateDt[6];
	SpiceDouble lt;
	spkezr_c(std::to_string(spiceID).c_str(), bt, "ECLIPJ2000", "NONE", soiID, stateBt, &lt);
	spkezr_c(std::to_string(spiceID).c_str(), rt, "ECLIPJ2000", "NONE", soiID, stateRt, &lt);
	stateChange(stateBt);
	stateChange(stateRt);

	if (soiID != "0") {
		for (int i = 0; i < 3; i++) {
			stateBt[i] += (*(gravSource->Pos))[i];
			stateRt[i] += (*(gravSource->Pos))[i];
		}
	}

	vertex_t dummy{};
	dummy.width = lWidth;
	dummy.col = lColor;

	int btIdx = closestVertex(lineBuffer, lineBufferSize, stateBt), 
		rtIdx = closestVertex(lineBuffer, -lineBufferSize, stateRt);
		*pbtIdx = btIdx; *prtIdx = rtIdx;

	for (int i = 0; i < REF_LIST_SIZE_U; i++) {
		spkezr_c(std::to_string(spiceID).c_str(), bt + rfDt * (i + 1), "ECLIPJ2000", "NONE", soiID, stateDt, &lt);
		stateChange(stateDt);
		if (soiID != "0") {
			for (int i = 0; i < 3; i++) {
				stateDt[i] += (*(gravSource->Pos))[i];
			}
		}
		dummy.pos = glm::vec3(stateDt[0], stateDt[1], stateDt[2]);
		refinedList[2 * i] = dummy; // plants idx 0,2,4,6
		refinedList[2 * i + 1] = dummy; // plants 1, 3, 5
	}
}