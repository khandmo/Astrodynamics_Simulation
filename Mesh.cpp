#include "Mesh.h"

#define UTC2J2000	946684800

Mesh::Mesh(const char* objName, std::vector <Vertex> vertices, std::vector <GLuint> indices, std::vector <Texture> textures, bool isLight, bool areRings, glm::vec4 objColor, glm::vec3 objPos, Shader *shaderProgram, int baryIDx, int spiceIDx){
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
	// PM position only helpful w/ planet rotation speed
	
	/*
	If generate rotation with approximation, need to pass dt to Renderer Move that
	calls for this function on each loop
	*/
	SpiceDouble w[3];
	SpiceInt dim;
	bodvcd_c(spiceID, "PM", 3, &dim, w); // this angle is equivalent to the rotation angle, no need for rotation rates to be preprogrammed

	float angle = glm::radians((float)w[0] + w[1] * (UTCtime / 86400)) + 1;
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

