#version 330 core
layout (location = 0) in vec3 aPos; // vector pos
layout (location = 1) in vec3 aNormal; // normals info
layout (location = 2) in vec3 aColor; // color info
layout (location = 3) in vec2 aTexCoord; // texture info

out vec3 crntPos; // outputs current pos to frag
out vec3 normal; // outputs normal to frag
out vec3 color; // output color to frag shader
out vec2 texCoord; // output texture to frag shader
out vec4 FragPosLightSpace; // output shadow info

// import camera matrix from main
uniform mat4 camMatrix;
// import model matrix from main
uniform mat4 model;
// shadow light space matrix
uniform mat4 lightSpaceMatrix;


void main(){ 
	// calc crnt pos and modified based on model updating
	crntPos = vec3(model * vec4(aPos, 1.0f));

	// output coords of all vertices
	gl_Position = camMatrix * vec4(crntPos, 1.0);

	// assigns norms from vertex data and modifies based on model rotation updating
	normal = mat3(model) * aNormal;

	// assigns color from vertex data
	color = aColor; 

	// assigns textures from vertex data
	texCoord = aTexCoord; 

	FragPosLightSpace = lightSpaceMatrix * vec4(crntPos, 1.0);
}



/*
This is a simple shader that just forwards the 3-vector
information to the needed 4-vector. The job of the
vertex shader is usually to normalize the coords
so they fit on the screen and then forward the new coords

shaderClass.cpp then compiles the shader (it takes this file
and the frag file)


The vertex shader should receive input, because of it's
position in the data flow, directly from the vertex data.
In this way, shader input should flow from the data to
here - the vertex shader, then to the fragment shader for
coloring. This is done using "in" and "out".

For layout location = 0, input is taken IN from the
3-vector aPos from each vertex in the set vertices

To send data from one shader to the other an output
must be declared to send the shader and a matching
input must exist in the receiving shader.

Since the fragment shader takes a 4-vector, one must
be initialized here with the calculated opacity w pos
and sent to the fragment shader.


Uniforms are a way to send data from the CPU to shaders
on the GPU and are unqiue for each shader program object
and can be accessed from any shader. They keep their values
until reset or updated. Uniforms are declared the same
way as "in" and "out"
: uniform (vector type) (vector name);
: in (vector type) (vector name);
: out (vector type) (vector name);
Since this vertex shader also manipulates where on the 
screen the vertex appears, uniforms can be used to 
transform the vertex matrix by some scale amount designated
in the main file AS WELL AS modifying colors by some
scale amount or by specific amounts if multiple uniforms
are made for each degree of freedom.

One can also send data about colors from the vertex matrix
but for this file to handle that, a layout attribute
must be added to expect more data. The VBO has to be changed
too now.
*/