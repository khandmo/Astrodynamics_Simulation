#version 330 core

// outputs colors
out vec4 FragColor;

// import current pos from vertex shader
in vec3 crntPos;
// import norms from vertex shader
in vec3 normal;
// import color from vertex shader
in vec3 color;
// import textures from vertex shader
in vec2 texCoord;
// import shadow info from vertex shader
in vec4 FragPosLightSpace;



// get texture units from main (up to 16)
uniform sampler2D diffuse0;
// get shadow map
uniform sampler2D shadowMap;


// get color of light from main
uniform vec4 lightColor;
// get position of light from main
uniform vec3 lightPos;

float ShadowCalculation(vec4 fpls){
	//perspective divide
	vec3 projCoords = fpls.xyz / fpls.w;
	//transform to range [0,1]
	projCoords = projCoords * 0.5 + 0.5;

	// closest depth value
	float closestDepth = texture(shadowMap, projCoords.xy).r;

	// current depth value
	float currentDepth = projCoords.z;

	vec3 normal = normalize(normal);
	vec3 lightDirection = normalize(lightPos - crntPos);
	float bias = max(0.05 * (1.0 - dot(normal, lightDirection)), 0.005);  // bias solution for shadow acne

	float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;    
	return shadow;
}


// for light attenuation 
float constant = 1;
float linear = 0.000001;
float quadratic = 0.000000001;


void main(){
	vec3 normal = normalize(normal);
	vec3 lightDirection = normalize(lightPos - crntPos);
	
	float distance = length(lightPos - crntPos);
	float attenuation = 1.0 / (constant + linear * distance +
						quadratic * (distance * distance));
	
	float diffuse = max(dot(normal, lightDirection), 0.0f) * attenuation;
	
	float shadow = ShadowCalculation(FragPosLightSpace);

	float ambient = 0.05;

	// assigns specific texture to sampler to frag shader
	FragColor = texture(diffuse0, texCoord) * lightColor * diffuse * ((1.0 - shadow) + ambient);
}

/*
Fragment shader does color processing and so an RGB value
is input. the fourth float indicates opacity

Without a way to input manual numbers here, every object
that uses this file will be this specific color

The input here comes from the vertex shader, which must
have a matching "out" vector as this one has an "in" vector
Again, the frag shader requires a 4-vector, whereas
the vertex shader requires a 3-vector.

Using a uniform vector instead means that colors can be
received from the main cpp file instead of set arbitrarily
here
*/