#version 330
  
layout(location = 0) in  vec3 vPosition; 
layout(location = 1) in  vec3 vNormal; 
layout(location = 2) in  vec2 vTexCoords;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projMatrix;

uniform float rho;
uniform vec4 F;
uniform vec4 C;

out vec3 rhoV;   //rho * V
out float F_dot_V; 
out float F_dot_P; 

out vec2 TexCoords;

void main() 
{ 
	TexCoords = vTexCoords;    

	vec4 P = modelMatrix * vec4(vPosition, 1);
	gl_Position = projMatrix * viewMatrix * P;

	vec3 V = C.xyz - P.xyz;

	rhoV = rho * V;
	F_dot_V = dot(F, vec4(V.xyz , 1.0));
	F_dot_P = dot(F, P);
} 
