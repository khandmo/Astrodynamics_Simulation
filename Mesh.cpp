#include "Mesh.h"

#define UTC2J2000	946684800

double distanceFind(std::vector<double> pt1, std::vector<double> pt2);

Mesh::Mesh(const char* objName, std::vector <Vertex> vertices, std::vector <GLuint> indices, std::vector <Texture> textures, bool isLight, bool areRings, glm::vec4 objColor, glm::vec3 objPos, Shader* shaderProgram, int baryIDx, int spiceIDx, double UTCtime) {
	Mesh::name = objName;
	Mesh::vertices = vertices;
	Mesh::indices = indices;
	Mesh::textures = textures;
	Mesh::areRings = areRings;
	Mesh::isLightSource = isLight;
	Mesh::Pos = objPos;
	Mesh::oPos = Pos;
	Mesh::spiceID = spiceIDx;
	Mesh::baryID = baryIDx;

	// initialize all kernels here, once
	furnsh_c("spice_kernels/de432s.bsp"); // for pos/vel of bodies (baryID)
	furnsh_c("spice_kernels/pck00011.tpc.txt"); // for axial orientation (spiceID)

	// set object light emission color if any
	if (isLight == true) {
		Mesh::Color = objColor;
	}

	// set object model position
	glm::mat4 objModel = glm::mat4(1.0f);
	objModel = glm::translate(objModel, Pos);
	Mesh::Model = objModel;


	// set object shader program
	Mesh::ShaderProgram = *shaderProgram;

	// vertex attribute object to send to GPU - array of attributes or states of vertices which package VBO's to send to GPU
	VAO.Bind();

	// vertex buffer object to send to GPU - array of data on vertices to be sent to GPU
	VBO VBO(vertices);

	// element buffer object is an array of pointers in data in VBO allows me to arrange pointers as i please instead of copying data repeatedly
	EBO EBO(indices);

	//Link VAO to VBO
	//parameters: VBO, which vertex attribute (layout) to configure, number of vertex attributes (1 per 3-vertex), type - of vector data stored, stride - space in memory between consecutive vertex attribs, offset - of where pos data begins in buffer

	// Links VBO to Position
	VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
	// Links to Normals (layout 1 for shaders, offset by 3 from the start of pos
	VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
	// Links to Colors (layout 2 for shader, offset by 6 from start of pos
	VAO.LinkAttrib(VBO, 2, 3, GL_FLOAT, sizeof(Vertex), (void*)(6 * sizeof(float)));
	// Links Texture (layout 3 for shaders, offset by 9 from the start of pos
	VAO.LinkAttrib(VBO, 3, 2, GL_FLOAT, sizeof(Vertex), (void*)(9 * sizeof(float)));
	VAO.Unbind();
	VBO.Unbind();
	EBO.Unbind();

	// Determine set of points for orbital path drawing and set them in a VBO for each Mesh to hold
	// uniquely

	// need to detect a full orbit (return to pt OR orbital period calculation)
	// return to pt would work for spacecraft imp too, worth trying

	// conversions
	int hour = 60 * 60;
	int day = hour * 24;
	int month = day * 30;
	int passed = 0;
	int hit = 0;
	int done = 0;
	int distance = 0; // best distance to initial pt
	double newD;

	if (name != "sun") {
		SpiceDouble state[6], newState[6];
		SpiceDouble lt;
		double et = UTCtime - UTC2J2000; // UTC to J2000
		spkezr_c(std::to_string(baryID).c_str(), et, "J2000", "NONE", "10", state, &lt);
		std::vector<double> initPos = { state[0], state[1], state[2] };

		// should transcribe the points to be used and the parameters (so the code will change automatically upon revision)
		// into a .txt file to read from since these values don't change, just generate them from number of orbital subdivisions

		while (!done) {
			if (!passed) {
				et += month;
				spkezr_c(std::to_string(baryID).c_str(), et, "J2000", "NONE", "10", newState, &lt);
				newD = distanceFind(initPos, std::vector<double> { newState[0], newState[1], newState[2] });
				if (newD < distance) {
					passed = !passed;
				}
				distance = newD;
			}
			else if (passed && !hit) {
				et += month;
				spkezr_c(std::to_string(baryID).c_str(), et, "J2000", "NONE", "10", newState, &lt);
				newD = distanceFind(initPos, std::vector<double> { newState[0], newState[1], newState[2] });
				if (newD > distance) {
					hit = !hit;
				}
				distance = newD;
			}
			else if (passed && hit) { // backstep doesn't occur enough times/satisfied early 
				et -= day;
				spkezr_c(std::to_string(baryID).c_str(), et, "J2000", "NONE", "10", newState, &lt);
				newD = distanceFind(initPos, std::vector<double> { newState[0], newState[1], newState[2] });
				if (newD > distance)
					done = !done;
			}
			distance = newD;
		}
		et += day; // corrects last period check
		// now have orbital period between et & UTCtime - UTC2J2000 that can be subdivided, pts added to vector and saved in a .txt file
		// path points have to be modified for distance proportions and axis modifcation!!!!!!!!
		int numPts = 200;
		numPathPoints = numPts;
		std::vector<glm::vec3> orbit;
		double tPt = UTCtime - UTC2J2000;
		// check for txt file existence, verify numPts is the same, if not overwrite txt file during for loop
		std::cout << "orbital period " << (et - tPt) / day << '\n';
		for (int i = 0; i < numPts; i++) {
			tPt += (et - UTCtime + UTC2J2000) / numPts;
			spkezr_c(std::to_string(baryID).c_str(), tPt, "J2000", "NONE", "10", newState, &lt);
			orbit.push_back(glm::vec3{ newState[0], newState[1], newState[2] });
			std::cout << tPt << "\t" << orbit.size() << '\n';
		}
		pathVBO = setLine(orbit); // set VBO with path list


		// might need to add the first pt at the end so that it connects
		// might need to extend the outer planets with orbits too large to calculate time-wise with simple predictive arcs
		// until emphersis data expires, though they'd have to propogate back in time too
		// can forsee saturn+ having this issue and should extend loops as far as possible
		// text file will shorten boot times
	}	
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
	glUniform3f(glGetUniformLocation((ShaderProgram).ID, "lightPos"), lightSource.Pos.x, lightSource.Pos.y, lightSource.Pos.z);

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
	bodvcd_c(spiceID, "PM", 3, &dim, w); // this angle is equivalent to the rotation angle, no need for rotation rates to be preprogrammed

	float angle = glm::radians((float)w[0] + w[1] * (UTCtime / 86400)) + 2; 
	// transforms model matrix by rotation
	Model = glm::rotate(Model, angle, Up);
	currAngleRad = angle;
		//2.0f * glm::pi<float>() * 0.0600068844f / 40; // arbitrary testing constant
	// redraws the shader for modified model
	dullShader(*lightSource);
}

void Mesh::AxialTilt(GLfloat tiltDeg) {
	SpiceDouble ra[3], dec[3], lambda[1];
	// the first constant in each of above is sufficient for the time scales of this program
	SpiceInt dim;
	bodvcd_c(spiceID, "POLE_RA", 3, &dim, ra);
	bodvcd_c(spiceID, "POLE_DEC", 3, &dim, dec);

	SpiceDouble range = 1, rectan[3];

	radrec_c(range, ra[0], dec[0], rectan); // turns ra & dec to rectangular coords
	double poleCoords[3];
	poleCoords[0] = rectan[1]; poleCoords[1] = rectan[2]; poleCoords[2] = rectan[0];
	glm::vec3 poleVec = glm::vec3(poleCoords[0], poleCoords[1], poleCoords[2]);
	poleVec = glm::rotateY(poleVec, glm::radians(-23.4f)); // north pole vector in correct frame


	Model = glm::rotate(Model, glm::radians(tiltDeg), glm::cross(Up, poleVec));
	axisTiltDegree = tiltDeg;
}

void Mesh::Orbit(Mesh* lightSource, double UTCtime) {

	// SPICE TO MODIFY Pos vector

	SpiceDouble state[6];
	SpiceDouble lt;
	double et = UTCtime - UTC2J2000; // UTC to J2000
	spkezr_c(std::to_string(baryID).c_str(), et, "J2000", "NONE", "10", state, &lt);

	// Positions scaled by largest possible distance from neptune to the sun
	double largestDistance = 4550000000;
	// use same formula for calculating proprtionate radii, recall OpenGL coordinate system is analogous to cartesian by (y, z, x)
	Pos.x = (state[0] * 6100 / largestDistance);
	Pos.y = (state[1] * 6100 / largestDistance);
	Pos.z = (state[2] * 6100 / largestDistance);

	// rotate positions on x-axis negative 23.4 degrees to get eccliptic as a flat plane
	glm::vec3 posVec = glm::vec3(Pos.x, Pos.y, Pos.z);
	posVec = glm::rotateX(posVec, glm::radians(-23.4f));
	Pos.x = posVec.y; Pos.y = posVec.z; Pos.z = posVec.x;



	// if object are rings, meshes normals must be flipped when the sun crosses the ring plane (z-axis turning points)
	if (areRings) { // below assumes the starting position is at z = 0
		if (Pos.z > 0 && !sign) { 
			sign = !sign;
			for (int i = 0; i < vertices.size(); i++) {
				vertices[i].normal = -vertices[i].normal;
			}
			// must bind new normals to the shaders
			VAO.Bind();
			VBO VBO(vertices);
			VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
			VAO.Unbind();
			VBO.Unbind();
		}
		else if (Pos.z < 0 && sign) {
			sign = !sign;
			for (int i = 0; i < vertices.size(); i++) {
				vertices[i].normal = -vertices[i].normal;
			}
			VAO.Bind();
			VBO VBO(vertices);
			VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
			VAO.Unbind();
			VBO.Unbind();
		}
	}

	updateModel(*gravSource);
	dullShader(*lightSource);	// ONLY TIME lightSource is used in this fxn

}

void Mesh::updateModel(Mesh& source) {
	// change model pos and rotation and axial tilt
	// orbit disappears the model
	glm::mat4 objModel = glm::mat4(1.0f);
	Model = glm::translate(objModel, Pos);

	Model = glm::rotate(Model, glm::radians(axisTiltDegree), glm::vec3(1.0f, 0.0f, 0.0f));
	Model = glm::rotate(Model, currAngleRad, Up);
}

double distanceFind(std::vector<double> pt1, std::vector<double> pt2) {
	double x2 = abs(pt1[0] - pt2[0]);
	double y2 = abs(pt1[1] - pt2[1]);
	double z2 = abs(pt1[2] - pt2[2]);
	x2 *= x2; x2 += (y2 * y2) + (z2 * z2);
	return sqrt(x2);
}