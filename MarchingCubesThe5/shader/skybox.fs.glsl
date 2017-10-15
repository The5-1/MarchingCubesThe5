#version 330
layout(location = 0)  out vec4 out0; // color 

uniform samplerCube cubemap;
 
in vec3 texCoord;

void main (void) {
    out0 = texture(cubemap, texCoord);
	//out0 = vec4(1.0f, 0.0f, 0.0f, 1.0f);
}
