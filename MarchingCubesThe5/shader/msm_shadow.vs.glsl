#version 330 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

uniform mat4 uMatP;
uniform mat4 uMatV;
uniform mat4 uMatM;
uniform mat4 uMatVP_Light;


out vec3 vPosM; //current Pixel World Space Position
out vec3 vNormalM; //curent Pixel Normal
out vec2 vUV0; //normal texture cooridnates for texture lookup
out vec4 vPosMVP_Light; //current Pixel Light Space Position

void main()
{    
	
    vPosM = vec3(uMatM * vec4(aPosition, 1.0));
    vNormalM = transpose(inverse(mat3(uMatM))) * aNormal;
    vUV0 = aTexCoords;

	//We transform the Positions into light space here in ther vertex shader, so we save massive performance in the fragment shader
    vPosMVP_Light = uMatVP_Light * vec4(vPosM, 1.0);

    gl_Position = uMatP * uMatV * vec4(vPosM, 1.0);
}