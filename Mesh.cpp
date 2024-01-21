#include "Mesh.h"

Mesh::Mesh(const char* objName, std::vector <Vertex>& vertices, std::vector <GLuint>& indices, std::vector <Texture>& textures, bool isLight, bool areRings, glm::vec4 objColor, glm::vec3 objPos, Shader* shaderProgram, GLfloat objMass){
	Mesh::name = objName;
	Mesh::vertices = vertices;
	Mesh::indices = indices;
	Mesh::textures = textures;
	Mesh::areRings = areRings;
	Mesh::isLightSource = isLight;
	Mesh::Pos = objPos;
	Mesh::oPos = Pos;
	Mesh::sphPos = Pos;
	Mesh::mass = objMass;

	// set object light emission color if any
	if (isLight == true) {
		Mesh::Color = objColor;
	}

	// set object model position
	glm::mat4 objModel = glm::mat4(1.0f);
	objModel = glm::translate(objModel, Pos); 
	Mesh::Model = objModel;

	// set object shader program
	Mesh::ShaderProgram = shaderProgram;

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
	// Links Texture (layout 3 for shaders, offset by 8 from the start of pos
	VAO.LinkAttrib(VBO, 3, 2, GL_FLOAT, sizeof(Vertex), (void*)(9 * sizeof(float)));
	VAO.Unbind();
	VBO.Unbind();
	EBO.Unbind();
}

void Mesh::setShadowShader(Shader& program, glm::mat4 lightSpaceMatrix) {
	shadowShaderProgram = &program;
	lsMatrix = lightSpaceMatrix;
}

void Mesh::switchShader() {
	if (shadowShaderProgram != NULL) {
		Shader* dummy = shadowShaderProgram;
		shadowShaderProgram = ShaderProgram;
		ShaderProgram = dummy;
	}
}

void Mesh::setDepthMap(GLuint depthMapInput) {
	depthMap = depthMapInput;
}

void Mesh::emissionShader() {
	(*ShaderProgram).Activate();
	// light shader position uniform
	glUniformMatrix4fv(glGetUniformLocation((*ShaderProgram).ID, "model"), 1, GL_FALSE, glm::value_ptr(Model));
	// light shader color uniform (sun on sun)
	glUniform4f(glGetUniformLocation((*ShaderProgram).ID, "lightColor"), Color.x, Color.y, Color.z, Color.w);
}

void Mesh::dullShader(Mesh& lightSource) {
	(*ShaderProgram).Activate();
	// model shader position uniform
	glUniformMatrix4fv(glGetUniformLocation((*ShaderProgram).ID, "model"), 1, GL_FALSE, glm::value_ptr(Model));
	// model shader color uniform (sun on earth) and shader uniform
	glUniform4f(glGetUniformLocation((*ShaderProgram).ID, "lightColor"), lightSource.Color.x, lightSource.Color.y, lightSource.Color.z, lightSource.Color.w);
	glUniform3f(glGetUniformLocation((*ShaderProgram).ID, "lightPos"), lightSource.Pos.x, lightSource.Pos.y, lightSource.Pos.z);

	// uniform for depth and default
	glUniformMatrix4fv(glGetUniformLocation((*ShaderProgram).ID, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lsMatrix));
	// uniform for default
	glUniform1i(glGetUniformLocation((*ShaderProgram).ID, "shadowMap"), 1); // maybe an issue here
}

void Mesh::Draw(Camera& camera) {

	(*ShaderProgram).Activate();
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
		textures[i].texUnit((*ShaderProgram), (type + num).c_str(), i);
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
	camera.Matrix((*ShaderProgram), "camMatrix");

	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}

void Mesh::Rotate(GLfloat angleRad, Mesh lightSource) {
	// transforms model matrix by rotation
	Model = glm::rotate(Model, angleRad, Up);
	currAngleRad += angleRad;
	// redraws the shader for modified model
	dullShader(lightSource);
}

void Mesh::AxialTilt(GLfloat tiltDeg) {
	// rotate model by cross product of Up and tiltDir 
	Model = glm::rotate(Model, glm::radians(tiltDeg), glm::vec3(1.0f, 0.0f, 0.0f));
	axisTiltDegree = tiltDeg;
}

void Mesh::Orbit(Mesh& source, Mesh& lightSource, glm::vec3 objVel, float dt) {
	// if object has fallen into the sun leave it there
	if (sphPos.x < 0) {
		h = 0;
		sphPos.x = 0;
	}

	long double c2 = pow(299792458, 2);
	// if initialization factors uninitilialized, calculate them
	if (redMass == 0) {
		std::cout << name << ": " << std::endl;
		Pos = Pos - source.Pos;
		sphPos = glm::vec3(glm::length(Pos), glm::acos(Pos.y / glm::length(Pos)), ((Pos.x / sqrt(Pos.x * Pos.x)) * glm::acos(Pos.z / sqrt(Pos.z * Pos.z + Pos.x * Pos.x))));
		redMass = (1.0f / ((1.0f / mass) + (1.0f / source.mass)));
		h = glm::length(glm::cross(Pos, (mass * objVel))) / redMass;
		std::cout << "h  is " << h << " L is " << h * mass << std::endl;
		std::cout << "objVelocity is " << length(objVel) << std::endl;
		std::cout << "stable circular angular velocity is " << sqrt(source.mass * sphPos.x) * redMass << std::endl; // angular velocity L
		epsilon = (3 * source.mass * source.mass) / (h * h);
		ecc = (((h * h) / (source.mass * sphPos.x)) - 1) / cos(sphPos.z * (1 - epsilon));
		std::cout << "eccentricity is " << ecc << std::endl;
	}

	// 2 - body relativistic equations
	float dPhi = h / (sphPos.x * sphPos.x);
	sphPos.z += dPhi / dt; // note sphPos.z does not reset after 2pi, it numerically continues to grow during the sim
	sphPos.x = pow((source.mass / (h * h)) * (1 + ecc * cos(sphPos.z * (1 - epsilon))), -1);

	// inclination implementation would involve sinusoidal variation of sphPos.y based on sphPos.z
	

	// turn back into cartesian
	Pos = glm::vec3(sphPos.x * sin(sphPos.y) * sin(sphPos.z), sphPos.x * cos(sphPos.y), sphPos.x * sin(sphPos.y) * cos(sphPos.z));
	Pos += source.Pos;

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
	
	updateModel(source);
	dullShader(lightSource);	
}

void Mesh::updateModel(Mesh& source) {
	// change model pos and rotation and axial tilt
	// orbit disappears the model
	glm::mat4 objModel = glm::mat4(1.0f);
	Model = glm::translate(objModel, Pos);

	Model = glm::rotate(Model, glm::radians(axisTiltDegree), glm::vec3(1.0f, 0.0f, 0.0f));
	Model = glm::rotate(Model, currAngleRad, Up);
}
