#ifndef OBJECT_H
#define OBJECT_H

#include "Mesh.h"

/*
Object Mesh Summon creates the vertex data and element index data for a given shaped object using algorithms from simple inputs.
Box creates a 6 sided cube, Rings creates a disk (higher numbers of division would increase curvature, Sphere creates a sphere (petals and breakpts would increase curvature).
*/

class Object {
public:

	std::vector <Vertex> vertices;
	std::vector <GLuint> indices;

	Object();

	// creates object box at origin painted in associated color
	void Box(GLfloat length, GLfloat red, GLfloat green, GLfloat blue);

	// creates planar object that
	void Rings(float innerRadius, float outerRadius, GLfloat red, GLfloat green, GLfloat blue);

	// creates object sphere with # of pedals 360 * factor360 painted in associated color
	void Sphere(float radius, GLfloat red, GLfloat green, GLfloat blue);
};

#endif
