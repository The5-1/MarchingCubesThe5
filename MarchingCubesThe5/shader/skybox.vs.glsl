#version 330
layout(location = 0) in  vec3 vPosition; 

uniform mat4 viewMatrix;
uniform mat4 projMatrix;
 
out vec3 texCoord;

void main() {

    texCoord = vPosition.xyz;

    gl_Position = projMatrix * viewMatrix * vec4(vPosition, 1.0);
}