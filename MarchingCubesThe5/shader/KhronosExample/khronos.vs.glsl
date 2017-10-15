/* *********************************************************************************************************
gl_InstanceID to get the Id of the current kernel
********************************************************************************************************* */

#version 330
  
layout(location = 0) in vec3 vPosition; 
layout(location = 1) in vec2 vUVS;
layout(location = 2) in vec3 vOffset;


uniform mat4 viewMatrix;
uniform mat4 projMatrix;


out vec3 color;

void main() {
	color = vec3(0.0, 1.0, 0.0);
	gl_Position = projMatrix * viewMatrix * vec4((vOffset + vPosition), 1.0);
}
