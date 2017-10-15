#version 330 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
} fs_in;

uniform sampler2D texture_diffuse;
uniform sampler2D shadowMap;

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


float ShadowCalculation(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
	//we get the current fragments light space position (transformation happened in vertex shader!)

	//1.) converte the Light-Space positions back into Texture coordinates so we can get the depth from the light depth buffer
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;

	//2.) grab the depth we stored in the light depth buffer
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float depth_light = texture(shadowMap, projCoords.xy).r; 

	//3.) grab the depth our pixel would have in light space
    // get depth of current fragment from light's perspective
    float depth_fragment = projCoords.z;

    // check whether current frag pos is in shadow
	//add some bias to fight "shaodw acne"

	float bias = max(0.0001 * (1.0 - dot(normal, lightDir)), 0.0001);  
    float shadow = depth_fragment - bias > depth_light  ? 1.0 : 0.0;

	if (projCoords.x < 0 || projCoords.x > 1 || projCoords.y < 0 || projCoords.y > 1 ) {shadow = 1.0;}

	//return projCoords.x*projCoords.y;
    
	return shadow * (1.0-zeroOutsideUV(projCoords.xy));
}

void main()
{           
    vec3 color = texture(texture_diffuse, fs_in.TexCoords).rgb;
    vec3 normal = normalize(fs_in.Normal);
    vec3 lightColor = vec3(1.0);

    // ambient
    vec3 ambient = 0.15 * color;

    // diffuse
    vec3 lightDir = normalize(lightPos - fs_in.FragPos);
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * lightColor;

    // specular
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);
    float spec = 0.0;
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
    vec3 specular = spec * lightColor;  
	  
    // calculate shadow
    float shadow = ShadowCalculation(fs_in.FragPosLightSpace,normal,lightDir);       
    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular)) * color;    
    
    FragColor = vec4(lighting, 1.0);
}