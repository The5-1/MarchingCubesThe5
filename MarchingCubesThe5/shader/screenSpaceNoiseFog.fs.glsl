#version 330
layout(location = 0)  out vec4 out0; // color 

uniform sampler2D diffuseTex;
uniform sampler2D positionTex;

uniform vec3 cameraPosition;
uniform float fogDensity;
uniform float c;
uniform float time;

uniform vec2 resolution;
uniform float density;

in vec2 texCoords;

#define fogColor vec3(0.5,0.6,0.7)

/* //////////////////////////////////////////////////////////////////////
	Noise
  ////////////////////////////////////////////////////////////////////// */ 
float random (vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898,78.233)))*43758.5453123);
}

float noise (in vec2 st) {
    vec2 i = floor(st);
    vec2 f = fract(st);

    // Four corners in 2D of a tile
    float a = random(i);
    float b = random(i + vec2(1.0, 0.0));
    float c = random(i + vec2(0.0, 1.0));
    float d = random(i + vec2(1.0, 1.0));

    // Smooth Interpolation

    // Cubic Hermine Curve.  Same as SmoothStep()
    vec2 u = f*f*(3.0-2.0*f);
    // u = smoothstep(0.,1.,f);

    // Mix 4 coorners porcentages
    return mix(a, b, u.x) + 
            (c - a)* u.y * (1.0 - u.x) + 
            (d - b) * u.x * u.y;
}

float valueNoise(){
	 vec2 st = gl_FragCoord.xy/resolution.xy;

	 //st = (st-vec2(5.0))*(abs(sin(time*0.2))*5.);
     st.x += time * 0.5;

    // Scale the coordinate system to see
    // some noise in action
    vec2 pos = vec2(st * density);

    // Use the noise function
    float n = noise(pos);

	return n;
}

/* //////////////////////////////////////////////////////////////////////
	Fog
  ////////////////////////////////////////////////////////////////////// */ 
vec3 nonConstantDensityFog(){
	//Variables
	vec4 diffuse = texture2D(diffuseTex, texCoords);
	vec3 worldPosition =  texture2D(positionTex, texCoords).xyz;

	float camY = cameraPosition.y;
	float worldY = worldPosition.y;
	float lengthY = abs(worldPosition.y - cameraPosition.y);
	float directionY = (worldPosition.y - cameraPosition.y) / lengthY;
	
	float fogAmount = c * exp(-camY * fogDensity) * ( (1.0 - exp( -lengthY * directionY * fogDensity )) / directionY );

	fogAmount = exp(-valueNoise()) * fogAmount;

	return mix( diffuse.xyz, fogColor, fogAmount);
	
} 

vec3 constantDensityFog(){
	//Variables
	vec4 diffuse = texture2D(diffuseTex, texCoords);
	vec3 worldPosition =  texture2D(positionTex, texCoords).xyz;

	float distance = distance(worldPosition, cameraPosition);
	float fogAmount = exp(-valueNoise()) * (1.0 - exp( -distance * fogDensity));
    
    return mix( diffuse.xyz, fogColor, fogAmount );
} 

/* //////////////////////////////////////////////////////////////////////
	Main-Function
  ////////////////////////////////////////////////////////////////////// */ 

void main()
{
	vec3 color;

	color = constantDensityFog();

	out0 = vec4(color, 1.0);
}
