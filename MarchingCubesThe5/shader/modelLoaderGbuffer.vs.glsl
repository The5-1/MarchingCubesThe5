#version 330 core
layout (location = 0) in vec3 vPosition;
layout (location = 1) in vec3 vNormal;
layout (location = 2) in vec2 vTexCoords;

uniform mat4 modelMatrix;
uniform mat4 projMatrix;
uniform mat4 viewMatrix;

out vec3 normal;
out vec4 pos_ec;
out vec2 TexCoords;

void main()
{
	mat3 normalMatrix = transpose(inverse(mat3(viewMatrix * modelMatrix)));

	normal = vNormal.xyz;
	pos_ec = modelMatrix * vec4(vPosition, 1);

    TexCoords = vTexCoords;   
	vec4 pvmPos = projMatrix * viewMatrix * modelMatrix * vec4(vPosition, 1.0);
    gl_Position = pvmPos;
}

