//https://mynameismjp.wordpress.com/2015/02/18/shadow-sample-update/
//https://github.com/TheRealMJP/Shadows/blob/master/Shadows/MSM.hlsl

#version 330 core
out vec4 FragColor;

in vec3 vPosM; //current Pixel World Space Position
in vec3 vNormalM; //curent Pixel vNormalM
in vec2 vUV0; //vNormalM texture cooridnates for texture lookup
in vec4 vPosMVP_Light; //current Pixel Light Space Position

uniform sampler2D texture_diffuse;
uniform sampler2D shadowMap_depth;

uniform vec3 lightPos;
uniform vec3 viewPos;

uniform float depthBias;
uniform float momentBias;

#define PI 3.14159265359

float mad(float mvalue, float avalue, float bvalue){
	return mvalue * avalue + bvalue;
}

vec4 calculateMoments(float _depth){
	float square = _depth * _depth;
	return vec4(_depth, square, square * _depth, square * square);
}

vec4 calculateTrigonometricMoments(float _depth){
	float trigoDepth = 2.0 * PI * _depth;
	float doubleTrigoDepth = 2.0 * trigoDepth;
	return vec4(sin(trigoDepth), cos(trigoDepth), sin(doubleTrigoDepth), cos(doubleTrigoDepth));
}

float ComputeMSMHamburger(vec4 moments, float fragmentDepth , float _depthBias, float _momentBias)
{
    // Bias input data to avoid artifacts
    vec4 b = mix(moments, vec4(0.5f, 0.5f, 0.5f, 0.5f), _momentBias);
    vec3 z;
    z[0] = fragmentDepth - _depthBias;

    // Compute a Cholesky factorization of the Hankel matrix B storing only non-
    // trivial entries or related products
    float L32D22 = mad(-b[0], b[1], b[2]);
    float D22 = mad(-b[0], b[0], b[1]);
    float squaredDepthVariance = mad(-b[1], b[1], b[3]);
    float D33D22 = dot(vec2(squaredDepthVariance, -L32D22), vec2(D22, L32D22));
    float InvD22 = 1.0f / D22;
    float L32 = L32D22 * InvD22;

    // Obtain a scaled inverse image of bz = (1,z[0],z[0]*z[0])^T
    vec3 c = vec3(1.0f, z[0], z[0] * z[0]);

    // Forward substitution to solve L*c1=bz
    c[1] -= b.x;
    c[2] -= b.y + L32 * c[1];

    // Scaling to solve D*c2=c1
    c[1] *= InvD22;
    c[2] *= D22 / D33D22;

    // Backward substitution to solve L^T*c3=c2
    c[1] -= L32 * c[2];
    c[0] -= dot(c.yz, b.xy);

    // Solve the quadratic equation c[0]+c[1]*z+c[2]*z^2 to obtain solutions z[1] and z[2]
	//Use the p-q-Formula, e.g. z^2 + pz + q with z1 = -p/2 + sqrt( (p/2)^2 - q), z2 = -p/2 - sqrt( (p/2)^2 - q)
    float p = c[1] / c[2];
    float q = c[0] / c[2];
    float D = (p * p * 0.25f) - q;
    float r = sqrt(D);
    z[1] =- p * 0.5f - r;
    z[2] =- p * 0.5f + r;

    // Compute the shadow intensity by summing the appropriate weights
    vec4 switchVal = (z[2] < z[0]) ? vec4(z[1], z[0], 1.0f, 1.0f) :
                      ((z[1] < z[0]) ? vec4(z[0], z[1], 0.0f, 1.0f) :
                      vec4(0.0f,0.0f,0.0f,0.0f));

    float quotient = (switchVal[0] * z[2] - b[0] * (switchVal[0] + z[2]) + b[1])/((z[2] - switchVal[1]) * (z[0] - z[1]));
    float shadowIntensity = switchVal[2] + switchVal[3] * quotient;
    return 1.0f - clamp(shadowIntensity, 0.0, 1.0);
}

void main()
{           
    vec3 color = texture(texture_diffuse, vUV0).rgb;
    vec3 vNormalM = normalize(vNormalM);
    vec3 lightColor = vec3(1.0);

	//Get depth from texture
	vec3 projCoords = vPosMVP_Light.xyz / vPosMVP_Light.w;
    projCoords = projCoords * 0.5 + 0.5;
    float depth_light = texture(shadowMap_depth, projCoords.xy).r; 
    float depth_fragment = projCoords.z;

	vec4 moments = calculateMoments(depth_light);
	float shadow_moment = ComputeMSMHamburger(moments, depth_fragment , depthBias, momentBias);

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

	//finished lighting 
	vec3 lighting = (ambient + shadow_moment * (diffuse + specular)) * color;  
	  
    FragColor = vec4(lighting, 1.0);
}