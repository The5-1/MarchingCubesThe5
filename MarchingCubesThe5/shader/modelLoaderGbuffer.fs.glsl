#version 330 core

uniform sampler2D texture_diffuse1;

in vec2 TexCoords;
in vec3 normal;
in vec4 pos_ec;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormal;
layout(location = 2) out vec4 outPos;

void main()
{    
	outColor = texture(texture_diffuse1, TexCoords);
	
	outNormal = vec4(normalize(normal), 1);
	
	outPos = vec4(pos_ec.xyz/pos_ec.w, 1);
}

