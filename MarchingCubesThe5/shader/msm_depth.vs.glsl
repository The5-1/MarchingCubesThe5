#version 330 core
layout (location = 0) in vec3 aPosition;
//layout (location = 1) in vec3 aNormal; //no need for scene depth, shadow bias happens via derivates
//layout (location = 2) in vec2 aTexCoords; //no textures needed to calculate scene depth

uniform mat4 uMatVP_Light; //ViewProjection
uniform mat4 uMatM;

//out vec3 vNormalMVP;
//out vec2 vUV0;

void main()
{
	//vUV0 = aTexCoords;
	
	//vNormalMVP = transpose(inverse(mat3(uMatM))) * aNormal;
	
    gl_Position = uMatVP_Light * uMatM * vec4(aPosition, 1.0);
} 