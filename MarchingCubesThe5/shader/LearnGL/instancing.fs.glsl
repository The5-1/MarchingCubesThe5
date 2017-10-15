#version 330
layout(location = 0)  out vec4 out0; // color 

in vec3 color; 

void main() 
{ 
	out0 = vec4(color, 1);
}
