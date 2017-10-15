#version 330 core
out vec4 FragColor;

in vec3 vPosM; //current Pixel World Space Position
in vec3 vNormalM; //curent Pixel vNormalM
in vec2 vUV0; //vNormalM texture cooridnates for texture lookup
in vec4 vPosMVP_Light; //current Pixel Light Space Position

uniform sampler2D texture_diffuse;
uniform sampler2D shadowMap_moments;
uniform sampler2D shadowMap_depth;

uniform vec3 lightPos;
uniform vec3 viewPos;


float gt(float x, float y) {
  return max(sign(x - y), 0.0);
}

float lt(float x, float y) {
  return max(sign(y - x), 0.0);
}

float zeroOutsideUV(vec2 UV)
{
	return gt(UV.x,1.0)*gt(UV.y,1.0)*lt(UV.x,0.0)*lt(UV.y,0.0);
}

float ShadowCalculation(vec4 vPosMVP_Light, vec3 vNormalM, vec3 lightDir)
{
	//we get the current fragments light space position (transformation happened in vertex shader!)

	//1.) converte the Light-Space positions back into Texture coordinates so we can get the depth from the light depth buffer
    // perform perspective divide
    vec3 projCoords = vPosMVP_Light.xyz / vPosMVP_Light.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;

	//2.) grab the depth we stored in the light depth buffer
    // get closest depth value from light's perspective (using [0,1] range vPosMLight as coords)
    float depth_light = texture(shadowMap_depth, projCoords.xy).r; 

	//3.) grab the depth our pixel would have in light space
    // get depth of current fragment from light's perspective
    float depth_fragment = projCoords.z;

    // check whether current frag pos is in shadow
	//add some bias to fight "shaodw acne"

	float bias = max(0.0001 * (1.0 - dot(vNormalM, lightDir)), 0.0001);  
    float shadow = depth_fragment - bias > depth_light  ? 1.0 : 0.0;

	if (projCoords.x < 0 || projCoords.x > 1 || projCoords.y < 0 || projCoords.y > 1 ) {shadow = 1.0;}

	//return projCoords.x*projCoords.y;
    
	return shadow * (1.0-zeroOutsideUV(projCoords.xy));
}


float getVarianceShadow(vec4 vPosMVP_Light, vec3 vNormalM, vec3 lightDir)
{
	//0.) vPosMVP_Light transformation happened in vertex shader already! --> Efficient!

	//1.) converte the Light-Space positions back into Texture coordinates so we can get the depth from the light depth buffer
    vec3 projCoords = vPosMVP_Light.xyz / vPosMVP_Light.w;
    projCoords = projCoords * 0.5 + 0.5;
	
	//2.) get all the depths
	float fragZ = projCoords.z; //current fragment depth
	if (projCoords.z < 0.0 || projCoords.x < 0 || projCoords.x > 1 || projCoords.y < 0 || projCoords.y > 1 ) {return 1.0;} //clip outside depth map borders //probably don't for pointlight shadows!
	
	float moment1 = texture(shadowMap_depth, projCoords.xy).x; //moment1 is regular depth
	vec3 moments = texture(shadowMap_moments, projCoords.xy).rgb; //other moments from our special FBO
	
	float variance = moments.x - (moment1*moment1);
	variance = max(variance,0.00002);

	//float bias = max(0.0001 * (1.0 - dot(vNormalM, lightDir)), 0.0001);
	//float d = fragZ - bias- moment1;
	
	float d = fragZ - moment1;
	float p_max = variance / (variance + d*d);
		
	return 1.0-p_max;
}



//single precision 16bit per moment is enough
//bias alpha of 2*10^-16 is NOT scene dependent, so always this!
//Legendre Moments
float getMomentShadow(vec4 vPosMVP_Light, vec3 vNormalM, vec3 lightDir)
{
	//0.) vPosMVP_Light transformation happened in vertex shader already! --> Efficient!

	//1.) converte the Light-Space positions back into Texture coordinates so we can get the depth from the light depth buffer
    vec3 projCoords = vPosMVP_Light.xyz / vPosMVP_Light.w;
    projCoords = projCoords * 0.5 + 0.5;
	
	//2.) get all the depths
	float fragZ = projCoords.z; //current fragment depth
	if (projCoords.z < 0.0 || projCoords.x < 0 || projCoords.x > 1 || projCoords.y < 0 || projCoords.y > 1 ) {return 1.0;} //clip outside depth map borders //probably don't for pointlight shadows!
	
	float moment1 = texture(shadowMap_depth, projCoords.xy).x; //moment1 is regular depth
	vec3 moments = texture(shadowMap_moments, projCoords.xy).rgb; //other moments from our special FBO
	
	float variance = moments.x - (moment1*moment1);
	variance = max(variance,0.00002);

	//float bias = max(0.0001 * (1.0 - dot(vNormalM, lightDir)), 0.0001);
	//float d = fragZ - bias- moment1;
	
	float d = fragZ - moment1;
	float p_max = variance / (variance + d*d);
		
	return 1.0-p_max;
}

void main()
{           
    vec3 color = texture(texture_diffuse, vUV0).rgb;
    vec3 vNormalM = normalize(vNormalM);
    vec3 lightColor = vec3(1.0);

    // ambient
    vec3 ambient = 0.15 * color;

    // diffuse
    vec3 lightDir = normalize(lightPos - vPosM);
    float diff = max(dot(lightDir, vNormalM), 0.0);
    vec3 diffuse = diff * lightColor;

    // specular
    vec3 viewDir = normalize(viewPos - vPosM);
    float spec = 0.0;
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    spec = pow(max(dot(vNormalM, halfwayDir), 0.0), 64.0);
    vec3 specular = spec * lightColor;  
	  
    // calculate shadow
    float shadow_normal = ShadowCalculation(vPosMVP_Light,vNormalM,lightDir);
    float shadow_variance = getVarianceShadow(vPosMVP_Light,vNormalM,lightDir);


	//finished lighting
    vec3 lighting = (ambient + (1.0 - shadow_variance) * (diffuse + specular)) * color;    
    
    FragColor = vec4(lighting, 1.0);
	
	//FragColor = vec4(0.0,shadow_normal,shadow_variance,1.0);
}