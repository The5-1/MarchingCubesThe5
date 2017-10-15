//https://mynameismjp.wordpress.com/2015/02/18/shadow-sample-update/
//https://github.com/TheRealMJP/Shadows/blob/master/Shadows/MSM.hlsl

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

uniform float alpha;

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

mat3 choleskyDecomposition(mat3 A) {
	int n = 3;
	mat3 ret = mat3(0.0f);
	for (int r = 0; r < n; r++)
		for (int c = 0; c <= r; c++)
		{
			if (c == r)
			{
				float sum = 0;
				for (int j = 0; j < c; j++)
				{
					sum += ret[c][ j] * ret[c][j];
				}
				ret[c][c] = sqrt(A[c][c] - sum);
			}
			else
			{
				float sum = 0;
				for (int j = 0; j < c; j++)
					sum += ret[r][j] * ret[c][j];
				ret[r][c] = 1.0 / ret[c][c] * (A[r][c] - sum);
			}
		}
	return ret;
}

vec3 choleskyEvaluation(mat3 A, vec3 b)
{
	// Ax = b -> LUx = b. Then y is defined to be Ux
	mat3 L = choleskyDecomposition(A);
	mat3 U = transpose(L);
	vec3 x;
	vec3 y;
	int n = 3;

	// Forward solve Ly = b
	for (int i = 0; i < n; i++)
	{
		y[i] = b[i];

		for (int j = 0; j < i; j++)
		{
			y[i] -= L[i][j] * y[j];
		}

		y[i] /= L[i][i];
	}

	// Backward solve Ux = y
	for (int i = n - 1; i >= 0; i--)
	{
		x[i] = y[i];

		for (int j = i + 1; j < n; j++)
		{
			x[i] -= U[i][j] * x[j];
		}

		x[i] /= U[i][i];
	}
	return x;
}

float getMomentShadow(vec4 vPosMVP_Light, vec3 vNormalM, vec3 lightDir)
{
	//0.) vPosMVP_Light transformation happened in vertex shader already! --> Efficient!

	//1.) converte the Light-Space positions back into Texture coordinates so we can get the depth from the light depth buffer
    vec3 projCoords = vPosMVP_Light.xyz / vPosMVP_Light.w;
    projCoords = projCoords * 0.5 + 0.5;
	
	
	float fragZ = projCoords.z; //current fragment depth

	if (projCoords.z < 0.0 || projCoords.x < 0 || projCoords.x > 1 || projCoords.y < 0 || projCoords.y > 1 ) {return 1.0;} //clip outside depth map borders //probably don't for pointlight shadows!
	
	//2.) get all the depths
	float moment1 = texture(shadowMap_depth, projCoords.xy).x; //moment1 is regular depth
	vec3 moments = texture(shadowMap_moments, projCoords.xy).rgb; //other moments from our special FBO

	vec4 b = vec4(moment1, moments.x, moments.y, moments.z);
	vec4 bNew = (1.0 - alpha) * b + alpha * vec4(0.5, 0.5, 0.5, 0.5);

	mat3 A = mat3(1.0, bNew.x, bNew.y,
					bNew.x, bNew.y, bNew.z,
					bNew.y, bNew.z, bNew.w); 

	vec3 choleskySolution = vec3(1.0, fragZ, fragZ*fragZ);

	vec3 c = choleskyEvaluation(A, choleskySolution);

	float z2 = (-c.y + sqrt(c.y*c.y - 4.0 * c.z * c.x))/ (2.0 * c.y);
	float z3 = (-c.y - sqrt(c.y*c.y - 4.0 * c.z * c.x))/ (2.0 * c.y);

	if(z2 > z3){
		float swap = z3;
		z3 = z2;
		z2 = swap;
	}

	if(fragZ <= z2){
		return 0.0;
	}
	else if(fragZ <= z3){
		return (fragZ * z3 - bNew.x * (fragZ + z3) + bNew.y) / ((z3 - z2) * (fragZ - z2));
	}
	else{
		return 1.0 - (z2*z3 - bNew.x*(z2+z3) + bNew.y)/((fragZ - z2) * (fragZ - z3));
	}
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
    //float shadow_normal = ShadowCalculation(vPosMVP_Light,vNormalM,lightDir);
    float shadow_moment = getMomentShadow(vPosMVP_Light,vNormalM,lightDir);


	//finished lighting
	//ToDo: Which one is right?
    vec3 lighting = (ambient + (1.0 - shadow_moment) * (diffuse + specular)) * color;    
	//vec3 lighting = (ambient + shadow_moment * (diffuse + specular)) * color;  
	  
    FragColor = vec4(lighting, 1.0);


	///////////////////////////////////////////////////////////
	// Test if Cholesky Works
	///////////////////////////////////////////////////////////
	//mat4 testMat = mat4(18.00000, 22.00000, 54.00000, 42.00000,
	//	22.00000, 70.00000, 86.00000, 62.00000,
	//	54.00000, 86.00000, 174.00000, 134.00000,
	//	42.00000, 62.00000, 134.00000, 106.00000);
	//vec4 xVec = vec4(0.5, 0.5, 0, 0.5);
	//vec4 bVec = testMat * xVec;
	//FragColor = choleskyEvaluation(testMat, bVec);
}