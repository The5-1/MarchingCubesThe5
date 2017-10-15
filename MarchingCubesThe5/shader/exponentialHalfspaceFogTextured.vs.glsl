#version 330
  
layout(location = 0) in  vec3 vPosition; 
layout(location = 1) in  vec3 vNormal; 
layout(location = 2) in  vec2 vTexCoords;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projMatrix;

uniform float k;            // (F dot C <= 0.0) 
uniform float a;
uniform vec4 F;
uniform vec4 C;

out float c1; // k * (F dot P + F dot C)
out float c2; // (1 - 2k) * (F dot P)
out float F_dot_V;
out vec3 aV;
out vec2 TexCoords;

void main() 
{ 
	TexCoords = vTexCoords;    

	vec4 P = modelMatrix * vec4(vPosition, 1);
	gl_Position = projMatrix * viewMatrix * P;

	vec3 V = C.xyz - P.xyz;

	aV = (a / 2.0) * V;
	F_dot_V = dot(F, vec4(V.xyz , 1.0));
	float F_dot_P = dot(F, P);
	float F_dot_C = dot(F, C);

	c1 = k * (F_dot_P + F_dot_C);
	c2 = (1.0 - 2.0 * k) * (F_dot_P);
} 
