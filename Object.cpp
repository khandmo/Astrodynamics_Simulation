#include "Object.h"

glm::vec3 sphToCart(glm::vec3 sphCoords);

Object::Object(){

}

void Object::Box(GLfloat length, GLfloat red, GLfloat green, GLfloat blue) {

	// Vertices

	std::vector<int> posOrder{ 1, 1, 1, -1, -1, 1, -1, -1 }; // for x and y values -> z is pos or neg depending on int pos
	std::vector<float> texOrder{ 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f }; // directly related to above order for texture coords

	Vertex boxV[24] = {}; // 8 vertices * 3 special normal coords each
	GLfloat scale = length / 2;
	int signCheck = -1;
	int index = 0;

	for (int i = 0; i < 2; i++) {
		signCheck = -signCheck;
		for (int j = 0; j < 4; j++) {
			// get x and y from order vector and modify z from pos 
			// run loop 3 times for x,0,0 - 0,y,0 - 0,0,z, dont have to be normalized for norms
			// get color from input and texture from texOrder
			
			glm::vec3 pos = glm::vec3(scale * posOrder[2*j], scale * posOrder[2*j + 1], scale * signCheck);
			glm::vec3 col = glm::vec3(red, green, blue);
			glm::vec2 tex = glm::vec2(texOrder[2*j], texOrder[2*j + 1]);

			// x norm
			glm::vec3 norm = glm::vec3(pos.x, 0.0f, 0.0f);
			boxV[index] = Vertex{ pos, norm, col, tex };
			index++;
			// y norm
			norm = glm::vec3(0.0f, pos.y, 0.0f);
			boxV[index] = Vertex{ pos, norm, col, tex };
			index++;
			// z norm
			norm = glm::vec3(0.0f, 0.0f, pos.z);
			boxV[index] = Vertex{ pos, norm, col, tex };
			index++;
		}
	}
	std::vector <Vertex> boxVert(boxV, boxV + sizeof(boxV) / sizeof(Vertex));
	Object::vertices = boxVert;


	// Indices (every new vertex is seperated by 3

	GLuint boxI[36] = { // 6 faces * 2 triangles each * 3 vertices each
		// for each vertex in boxV the norms are in x,y,z order - the front/back uses z, top/bottom uses y, right/left uses x
		2, 5, 8, 5, 8, 11, // front face (z, +2)
		14, 17, 20, 17, 20, 23, // back face (z, +2)
		1, 7, 19, 1, 13, 19, // top face (y, +1)
		4, 10, 22, 4, 22, 16, // bottom face (y, +1)
		0, 12, 3, 3, 15, 12, // right face (x, +0)
		6, 18, 9, 9, 21, 18 // left face (x, +0)
	};
	std::vector <GLuint> boxElem(boxI, boxI + sizeof(boxI) / sizeof(GLuint));
	Object::indices = boxElem;
}

// there is a somewhat noticable seam in the rings but my mental arithmetic can't explain why, the texures should always be within bounds
void Object::Rings(float innerRadius, float outerRadius, GLfloat red, GLfloat green, GLfloat blue) {
	// Vertices
	const int divisions = 512;
	Vertex ringV[6 * divisions] = {};
	// add outerRad, two innerRad, two outerRad, second innerRad, ...
	float constAngleDiv = (2 * glm::pi<float>()) / divisions;

	// r is given by appropriate radii, theta is 0, phi is the angles
	glm::vec3 vPos;
	glm::vec2 vTex;
	glm::vec3 vNorm = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 vCol = glm::vec3(red, green, blue);

	for (int i = 0; i < divisions; i++) {
		// outer point
		vPos = glm::vec3(outerRadius, glm::half_pi<float>(), i * constAngleDiv);
		vTex = glm::vec2(1.0f, 1.0f - (i / divisions));
		ringV[i * 6] = { vPos, vNorm, vCol, vTex };
		ringV[(i * 6) + 3] = ringV[i * 6];
		// two inner points
		vPos = glm::vec3(innerRadius, glm::half_pi<float>(), i * constAngleDiv);
		vTex = glm::vec2(0.0f, 1.0f - (i / divisions));
		ringV[(i * 6) + 1] = { vPos, vNorm, vCol, vTex };
		vPos = glm::vec3(innerRadius, glm::half_pi<float>(), (i + 1) * constAngleDiv);
		vTex = glm::vec2(0.0f, 1.0f - ((i + 1) / divisions));
		ringV[(i * 6) + 2] = { vPos, vNorm, vCol, vTex };
		ringV[(i * 6) + 5] = ringV[(i * 6) + 2];
		// second outer point
		vPos = glm::vec3(outerRadius, glm::half_pi<float>(), (i + 1) * constAngleDiv);
		vTex = glm::vec2(1.0f, 1.0f - ((i + 1) / divisions));
		ringV[(i * 6) + 4] = { vPos, vNorm, vCol, vTex };
	}

	for (int i = 0; i < 6 * divisions; i++) {
		ringV[i].position = sphToCart(ringV[i].position);
	}

	std::vector <Vertex> ringVert(ringV, ringV + sizeof(ringV) / sizeof(Vertex));
	Object::vertices = ringVert;

	// Indices
	// indices can be added in order
	GLuint ringI[6 * divisions] = {};
	for (int i = 0; i < 6 * divisions; i++) {
		ringI[i] = i;
	}
	std::vector <GLuint> ringElem(ringI, ringI + sizeof(ringI) / sizeof(GLuint));
	Object::indices = ringElem;
}

void Object::Sphere(float radius, GLfloat red, GLfloat green, GLfloat blue) {
	// Vertices
	const int petals = 96; 
	const int breakpts = 16; // number of theta divisions from pole to equator
	// axial vertex, 1st breakpt norms, middle breakpt norms, equator norms, all * 2
	const int arrayNum = 2 * (petals + (petals * 5) + (petals * 6 * (breakpts - 1)) + (petals * 3)); // buffer overrun
	Vertex sphV[arrayNum] = {};
	glm::vec3 vCol = glm::vec3(red, green, blue);
	glm::vec2 vTex = glm::vec2(0.0f, 0.0f); // leave blank for UV mapping
	
	// top cap (top point, straight out, phi angle, top point, phi angle, 2phi angle, ...)
	glm::vec3 vPos;
	glm::vec3 vNorm;
	int fIndex = 0;

	float theta = glm::half_pi<float>() / (breakpts + 1);
	const float thetaConstant = glm::half_pi<float>() / (breakpts + 1);
	float approxCir = 2 * radius * sin(glm::pi<float>() / petals) * petals; // approximate circumference of spheroid
	float lengthApproxCir = 2 * radius * sin(glm::half_pi<float>() / (breakpts + 1)) * (breakpts + 1);
	float phi = ((2 * glm::pi<float>()) / petals) * sin(theta); // phi constant (equator) * varying factor relying on theta
	float widthTri = 2 * radius * sin(phi / 2) / approxCir; // division forces value to be [0, 1]
	float widthMid = (1 - (petals * widthTri)) / petals; // 0 when widthTri * petals is approxCir - true at equator
	float hyp = 2 * radius * sin(thetaConstant / 2) / lengthApproxCir; // length of side of trapezoid - division factor forces value to be [0, 1/2]

	for (int i = 0; i < petals; i++) {
		vPos = glm::vec3(radius, 0, 0);
		vNorm = glm::vec3(1, glm::half_pi<float>() / (2 * (breakpts+1)), (2 * i * glm::pi<float>()) / petals); 
		vTex = glm::vec2(((2 * (i + 1)) - 1) / (float)(2 * petals), 1.0f);
		// bisect the theta for the angle away from axial vertex, three pts will share this norm so add them here for the whole top cap
		sphV[fIndex++] = Vertex{ vPos, vNorm, vCol, vTex }; // add top point first
		for (int j = 1; j < 3; j++) { // for two triangle endpoints that share the norm value
			vPos = glm::vec3(radius, theta, (2 * (i + j - 1) * glm::pi<float>()) / petals);
			// UV mapping calculations
			if (j == 1) {
				vTex = glm::vec2(((widthMid / 2) + (i * (widthTri + widthMid))), 1 - sqrt(hyp * hyp + widthTri * widthTri));
			}
			else {
				vTex = glm::vec2(((widthMid / 2) + (i * (widthTri + widthMid))) + widthTri, 1 - sqrt(hyp * hyp + widthTri * widthTri));
			}
			sphV[fIndex++] = Vertex{ vPos, vNorm, vCol, vTex }; // add outside top cap triangles
		}
	}

	// middle pts and equator
	glm::vec3 vPos2;
	glm::vec3 vPos3;
	glm::vec2 vTex2;
	glm::vec2 vTex3;
	for (int p = 0; p < petals; p++) {
		// for each pedal
		for (int t = 1; t < breakpts + 2; t++) {
			theta = (glm::half_pi<float>() * t) / (breakpts + 1);
			float thetaNext = (glm::half_pi<float>() * (t + 1)) / (breakpts + 1);
			if (t > 1) {
				// for each breakpt that isnt the first
				vPos = glm::vec3(radius, theta, (2 * p * glm::pi<float>()) / petals); // bottom left vertex - can get tex from previous
				vPos2 = glm::vec3(radius, theta, (2 * (p + 1) * glm::pi<float>()) / petals); // pedal + 1 - have to make tex ( bottom right )
				vPos3 = glm::vec3(radius, (glm::half_pi<float>() * (t - 1)) / (breakpts + 1), (2 * (p + 1) * glm::pi<float>()) / petals); // breakpt - 1, pedal + 1 - can get tex from previous
				// norm is avg
				vNorm = glm::vec3(radius, (vPos.y + vPos2.y + vPos3.y) / 3, (vPos.z + vPos2.z + vPos3.z) / 3);
				// add to vertex list
				vTex = sphV[fIndex - 1].tex;
				sphV[fIndex++] = { vPos, vNorm, vCol, vTex };
				vTex2 = sphV[fIndex - 1].tex;
				vTex2.x += widthTri;
				sphV[fIndex++] = { vPos2, vNorm, vCol, vTex2 };
				vTex3 = sphV[fIndex - 4].tex;
				sphV[fIndex++] = { vPos3, vNorm, vCol, vTex3 };
			}


			if (t < breakpts + 1) {
				// for each breakpt that isnt the last
				vPos = glm::vec3(radius, theta, (2 * p * glm::pi<float>()) / petals); // top left vertex - can get tex from previous 
				vPos2 = glm::vec3(radius, theta, (2 * (p + 1) * glm::pi<float>()) / petals); // pedal + 1 - can get tex from previous
				vPos3 = glm::vec3(radius, thetaNext, (2 * p * glm::pi<float>()) / petals); // breakpt + 1 - have to make tex ( bottom left )
				// norm is avg
				vNorm = glm::vec3(radius, (vPos.y + vPos2.y + vPos3.y) / 3, (vPos.z + vPos2.z + vPos3.z) / 3);
				// add to vertex list
				if (t == 1) { // tex for first trapezoids connected to triangles
					vTex = sphV[(p * 3) + 1].tex;
					vTex2 = sphV[(p * 3) + 2].tex;
				}
				else { // tex for the rest of trapezoids
					vTex = sphV[fIndex - 4].tex;
					vTex2 = sphV[fIndex - 2].tex;
				}
				sphV[fIndex++] = {vPos, vNorm, vCol, vTex};
				sphV[fIndex++] = {vPos2, vNorm, vCol, vTex2};
				// UV mapping calculations
				phi = ((2 * glm::pi<float>()) / petals) * sin(thetaNext);
				float oWidthTri;
				t == 1 ? oWidthTri = 2 * radius * sin(((2 * glm::pi<float>()) / petals) * sin(theta) / 2) : oWidthTri = widthTri;
				widthTri = 2 * radius * sin(phi / 2) / approxCir; // width of the base of each succesive trapezoid
				float leg = ((widthTri / 2) - (oWidthTri / 2)); // difference between trapezoid bases, pythag for the height of the trapezoid
				widthMid = (1 - (petals * widthTri)) / (petals); // width of space between trapezoids if shape was unrolled
				vTex3 = glm::vec2(((widthMid / 2) + (p * (widthTri + widthMid))), vTex2.y - sqrt((hyp * hyp) - (leg * leg)));
				sphV[fIndex++] = {vPos3, vNorm, vCol, vTex3};
			}		
		}
	}
	// above vertices identical but * pi 
	for (int i = 0; i < arrayNum / 2; i++) {
		// matching each in order from half (textures must be similarly transformed)		
		sphV[(arrayNum / 2) + i] = sphV[i];
		sphV[(arrayNum / 2) + i].position.y = sphV[i].position.y + glm::pi<float>(); 
		sphV[(arrayNum / 2) + i].normal.y = sphV[i].normal.y + glm::pi<float>();
		sphV[(arrayNum / 2) + i].tex.y = 1 - sphV[i].tex.y;	
		sphV[(arrayNum / 2) + i].tex.x = sphV[i].tex.x + 0.5;
	}

	// convert all to cartesian coords
	for (int i = 0; i < arrayNum; i++) {
		sphV[i].position = sphToCart(sphV[i].position);
		sphV[i].normal = sphToCart(sphV[i].normal);
	}

	// send vertex data to object
	std::vector <Vertex> sphVert(sphV, sphV + sizeof(sphV) / sizeof(Vertex));
	Object::vertices = sphVert;

	// Indices
	const int elemNum = 2 * (petals * (1 + (2 * breakpts)));
	GLuint sphI[arrayNum] = {};
	
	// all indices created in order
	for (int i = 0; i < arrayNum; i++) {
		sphI[i] = i;
	}
	// add indices to object mesh
	std::vector <GLuint> sphElem(sphI, sphI + sizeof(sphI) / sizeof(GLuint));
	Object::indices = sphElem;
}