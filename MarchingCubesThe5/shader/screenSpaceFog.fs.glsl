#version 330
layout(location = 0)  out vec4 out0; // color 

uniform sampler2D diffuseTex;
uniform sampler2D normalTex;
uniform sampler2D positionTex;
uniform sampler2D depthTex;

uniform mat4 projMatrix;

uniform vec3 cameraDirection;
uniform vec3 cameraPosition;

uniform float fogDensity;
uniform float c;

uniform int fogType;

uniform float time;

in vec2 texCoords;

#define lightDirection vec3(0.0, 1.0,0.0)
#define fogColor vec3(0.5,0.6,0.7)
#define lightColor vec3(1.0,0.9,0.7)
#define white vec3(1.0, 1.0, 1.0)

/*
	1. Simple fog, only uses density function
*/
vec3 constantDensityFog(){
	//Variables
	vec4 diffuse = texture2D(diffuseTex, texCoords);
	vec3 worldPosition =  texture2D(positionTex, texCoords).xyz;

	float distance = distance(worldPosition, cameraPosition);
	float fogAmount = 1.0 - exp( -distance * fogDensity);
    
    return mix( diffuse.xyz, fogColor, fogAmount );
} 

/*
	2. Simple fog, only uses density function. Simulates light from one direction
*/

vec3 constantDensityFogWithGlobalLight(){
	//Variables
	vec4 diffuse = texture2D(diffuseTex, texCoords);
	vec3 worldPosition =  texture2D(positionTex, texCoords).xyz;

	float distance = distance(worldPosition, cameraPosition);
	vec3 directionCamWorld = normalize(worldPosition - cameraPosition);

	float fogAmount = 1.0 - exp( -distance * fogDensity);
    float sunAmount = max( dot( directionCamWorld, lightDirection ), 0.0 );

	vec3 fogColorWithLight  = mix( fogColor,
                           lightColor,
                           sunAmount * 1.5);

    return mix(diffuse.xyz, fogColorWithLight, fogAmount );
} 

/*
	3. Fog from Crysis-Paper
	http://www.iquilezles.org/www/articles/fog/fog.htm
	https://stackoverflow.com/questions/21549456/how-to-implement-a-ground-fog-glsl-shader
*/

vec3 nonConstantDensityFog(){
	//Variables
	vec4 diffuse = texture2D(diffuseTex, texCoords);
	vec3 worldPosition =  texture2D(positionTex, texCoords).xyz;

	float camY = cameraPosition.y;
	float worldY = worldPosition.y;
	float lengthY = abs(worldPosition.y - cameraPosition.y);
	float directionY = (worldPosition.y - cameraPosition.y) / lengthY;
	
	float fogAmount = c * exp(-camY * fogDensity) * ( (1.0 - exp( -lengthY * directionY * fogDensity )) / directionY );

	return mix( diffuse.xyz, fogColor, fogAmount);
	
} 

/*
	3. Wave-Fog
	https://blog.demofox.org/2014/06/22/analytic-fog-density/
*/

float DefiniteIntegral (in float x, in float amplitude, in float frequency, in float motionFactor)
{
    // Fog density on an axis:
    // (1 + sin(x*F)) * A
    //
    // indefinite integral:
    // (x - cos(F * x)/F) * A
    //
    // ... plus a constant (but when subtracting, the constant disappears)
    //
    x += time * motionFactor;
    return (x - cos(frequency * x)/ frequency) * amplitude;
}
 
//=======================================================================================
float AreaUnderCurveUnitLength (in float a, in float b, in float amplitude, in float frequency, in float motionFactor)
{
    // we calculate the definite integral at a and b and get the area under the curve
    // but we are only doing it on one axis, so the "width" of our area bounding shape is
    // not correct.  So, we divide it by the length from a to b so that the area is as
    // if the length is 1 (normalized... also this has the effect of making sure it's positive
    // so it works from left OR right viewing).  The caller can then multiply the shape
    // by the actual length of the ray in the fog to "stretch" it across the ray like it
    // really is.

    return (DefiniteIntegral(a, amplitude, frequency, motionFactor) - DefiniteIntegral(b, amplitude, frequency, motionFactor)) / (a - b);
}
 
//=======================================================================================
float FogAmount (in vec3 src, in vec3 dest)
{
    float len = length(dest - src);
     
    // calculate base fog amount (constant density over distance)   
    float amount = len * fogDensity; //From link: fogDensity = 0.1
     
    // calculate definite integrals across axes to get moving fog adjustments
    float adjust = 0.0;
    adjust += AreaUnderCurveUnitLength(dest.x, src.x, 0.01, 0.6, 2.0);
    adjust += AreaUnderCurveUnitLength(dest.y, src.y, 0.01, 1.2, 1.4);
    adjust += AreaUnderCurveUnitLength(dest.z, src.z, 0.01, 0.9, 2.2);
    adjust *= len;
     
    // make sure and not go over 1 for fog amount!
    return min(c * (amount+adjust), 1.0);
}

vec3 nonConstantSinusDensityFog(){
	//Variables
	vec4 diffuse = texture2D(diffuseTex, texCoords);
	vec3 worldPosition =  texture2D(positionTex, texCoords).xyz;

	return mix( diffuse.xyz, fogColor, FogAmount(cameraPosition, worldPosition));
}


/* //////////////////////////////////////////////////////////////////////
	Main-Function
  ////////////////////////////////////////////////////////////////////// */ 

void main()
{
	vec3 color;

	if(fogType == 0){
		color = constantDensityFog();
	}
	else if(fogType == 1){
		color = constantDensityFogWithGlobalLight();
	}
	else if(fogType == 2){
		color = nonConstantDensityFog();
	}
	else if(fogType == 3){
		color = nonConstantSinusDensityFog();
	}
	out0 = vec4(color, 1.0);
}
