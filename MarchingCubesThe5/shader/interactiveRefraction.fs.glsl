#version 330

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projMatrix;
uniform mat4 normalMatrix;

uniform vec3 cameraPosition;

uniform vec3 albedoColor;
uniform vec3 ambientColor;

uniform vec3 lightColor;
uniform float specular;
uniform float glossiness;
uniform float metalness;

uniform sampler2D frontNormal;
uniform sampler2D frontDepth;
uniform sampler2D backNormal;
uniform sampler2D backDepth;
uniform sampler2D envDome;
uniform sampler2D envDomeDepth;

layout(location = 0)  out vec4 out0; // color 


in vec3 Normal;
in vec3 View;//this does not inerpolate correctly
in vec3 Light;
in vec3 Halfway;
in vec2 tc;
in vec3 posW;



//https://stackoverflow.com/questions/32227283/getting-world-position-from-depth-buffer-value
vec3 WorldPosFromDepth(sampler2D depthtex, vec2 UV) 
{
	float near = 0.0;
	float far = 100.0;

	float depth = texture(depthtex,UV).r;
    float linear = near * far / ((depth * (far - near)) - far);
    float z = depth * 2.0 - 1.0;

    vec4 clipSpacePosition = vec4(UV * 2.0 - 1.0, z, 1.0);
    vec4 viewSpacePosition = inverse(projMatrix) * clipSpacePosition;

    // Perspective division
    viewSpacePosition /= viewSpacePosition.w;

    vec4 worldSpacePosition = inverse(viewMatrix) * viewSpacePosition;

    return worldSpacePosition.xyz;
}



//https://www.opengl.org/discussion_boards/showthread.php/173019-View-Space-Light-Position-Moving?p=1212628&viewfull=1#post1212628
vec3 PositionFromDepth_DarkPhoton(sampler2D depthtex, vec2 UV)
{
	float depth = texture(depthtex,UV).r;

	vec2 ndc;             // Reconstructed NDC-space position
	vec3 eye;             // Reconstructed EYE-space position
	
	float near = 0.1;
	float far = 100.0;
	float widthInv = 1.0/1024.0;
	float heightInv = 1.0/768.0;
	float top = 1.0;
	float bottom = 0.0;
	float left = 0.0;
	float right = 1.0;
	
	eye.z = near * far / ((depth * (far - near)) - far);

	ndc.x = ((gl_FragCoord.x * widthInv) - 0.5) * 2.0;
	ndc.y = ((gl_FragCoord.y * heightInv) - 0.5) * 2.0;

	eye.x = ( (-ndc.x * eye.z) * (right-left)/(2*near)
			- eye.z * (right+left)/(2*near) );
			
	eye.y = ( (-ndc.y * eye.z) * (top-bottom)/(2*near)
			- eye.z * (top+bottom)/(2*near) );

	return eye;
}


float Phong_diffuse(vec3 n, vec3 l)
{
	return max(0.0,dot(n,l));
}


float Phong_Specular(vec3 n, vec3 h)
{
	return pow(max(0.0, dot(n,h)), glossiness) * specular;
}

float distanceSquared(vec2 a, vec2 b) { a -= b; return dot(a, a); }

//vec3 2D_Raytrace(vec3 ray)
//{
//
//}

vec4 SSR(vec3 view, vec3 normal)
{
	vec3 refl = reflect(-view, normal); //negative view vector!!!
	//return vec4(view*2000.0,1.0); //triangle interpolation artifacts here!!!

	float maxDistance = 350.0; //in World Units
	float nearPlaneZ = -1.0; //negative NEAR
	float stride = 2.0; //int >= 1 //pixel step size
	float jitter = 0.0; //[0-1]
	float maxSteps = 256.0;
	float zThickness = 1.0; //thickness of Z-buffer layer in z-buffer units
	vec2 csZBufferSize = vec2(1024.0,768.0);
	vec2 resInv = 1.0/csZBufferSize;
		
	//vec3 csOrig = vec4(viewMatrix * vec4(posW,1.0)).xyz;
	//vec3 csOrig = vec4(viewMatrix * vec4(texture(uTexGbuffer_WorldPos,gl_FragCoord.xy*resInv).xyz,1.0)).xyz;
	//return csOrig;
	
	vec3 csOrig = vec4(viewMatrix * vec4(posW,1.0)).xyz;
	//return vec4(csOrig*2000.0,1.0); //no triangle artifact
	

	vec3 csDir = vec4(viewMatrix * vec4(refl,0.0)).xyz; //when transforming a direction rather than a point, w=0!!!
	
	// Clip to the near plane  
	float rayLength = ((csOrig.z + csDir.z * maxDistance) > nearPlaneZ) ? (nearPlaneZ - csOrig.z) / csDir.z : maxDistance;
	vec3 csEndPoint = csOrig + csDir * rayLength;
	//return vec4(csEndPoint*2000.0,1.0);
 
	// Project into homogeneous clip space // Project into screen space //A projection matrix that maps to pixel coordinates (not [-1, +1] normalized device coordinates)
	// F: so the regular projection matrix before dividing by w?
	vec4 H0 = projMatrix * vec4(csOrig, 1.0);
	vec4 H1 = projMatrix * vec4(csEndPoint, 1.0);
	float k0 = 1.0 / H0.w, k1 = 1.0 / H1.w;
 
	// The interpolated homogeneous version of the camera-space points
	// Switch the original points to values that interpolate linearly in 2D
	// later a k will be incremented inside the search loop and the actual ViewSpace position retrieved from that k afterwards
	vec3 Q0 = csOrig * k0;
	vec3 Q1 = csEndPoint * k1;
	
	// Screen-space endpoints
	vec2 P0 = H0.xy * k0*0.5 + 0.5;
	//P0.y = 1.0-P0.y;
	P0 *= csZBufferSize; //F: further down there is a division by the difference, if those coordinates are [0,1] that division will instead increase the value, so i guess i need pixel coordinates
	vec2 P1 = H1.xy * k1*0.5 + 0.5;
	//P1.y = 1.0-P1.y;
	P1 *= csZBufferSize;
	
	//return abs(vec3(H0));
	
	// If the line is degenerate, make it cover at least one pixel
	// to avoid handling zero-pixel extent as a special case later
	P1 += vec2((distanceSquared(P0, P1) < 0.0001) ? 0.01 : 0.0);
	
	vec2 delta = P1 - P0;
	
	//return abs(vec3(vec2(delta),0.0));
	
	// Permute so that the primary iteration is in x to collapse all quadrant-specific DDA cases later
	// Permute so that the primary iteration is in x to reduce large branches later
	
	bool permute = false;
	if (abs(delta.x) < abs(delta.y)) {
		// More-vertical line. Create a permutation that swaps x and y in the output
		permute = true;
		// Directly swizzle the inputs
		delta = delta.yx;
		P1 = P1.yx;
		P0 = P0.yx;        
	}
	
	// From now on, "x" is the primary iteration direction and "y" is the secondary one
	float stepDir = sign(delta.x);
	float invdx = stepDir / delta.x; //F: This here could be a problem when not using pixel coordinates, dividing by something < 1.0!
	vec2  dP = vec2(stepDir, delta.y * invdx);
 
	// Track the derivatives of Q and k
	vec3  dQ = (Q1 - Q0) * invdx;
	float dk = (k1 - k0) * invdx;
	

	// Scale derivatives by the desired pixel stride and then
	// offset the starting values by the jitter fraction
	dP *= stride; 
	dQ *= stride; 
	dk *= stride;
	P0 += dP + dP * jitter; 
	Q0 += dQ + dQ * jitter; 
	k0 += dk + dk * jitter;
 
	// Slide P from P0 to P1, (now-homogeneous) Q from Q0 to Q1, and k from k0 to k1
	vec3 Q = Q0; 
	float k = k0;
	
	//return vec4(Q*2000.0,1.0); //triangle interpolation artifact visible already
 
	// Adjust end condition for iteration direction
	// P1.x is never modified after this point, so pre-scale it by 
	// the step direction for a signed comparison
	float  end = P1.x * stepDir;

	// We track the ray depth at +/- 1/2 pixel to treat pixels as clip-space solid 
	// voxels. Because the depth at -1/2 for a given pixel will be the same as at 
	// +1/2 for the previous iteration, we actually only have to compute one value 
	// per iteration.
	float stepCount = 0.0;
	float prevZMaxEstimate = csOrig.z;
	float rayZMin = prevZMaxEstimate;
	float rayZMax = prevZMaxEstimate;
	float sceneZMax = rayZMax + 100.0;
	
	
	// We only advance the z field of Q in the inner loop, since Q.xy is never used until after the loop terminates
	// after the loop Q is homognized again using the k incremented in the loop
	
	vec2 hitPixel = vec2(-1.0);
	vec3 hitPoint = vec3(-1.0);
	
	for (vec2 P = P0; 
		 ((P.x * stepDir) <= end) && (stepCount < maxSteps) && ((rayZMax < sceneZMax - zThickness) || (rayZMin > sceneZMax)) && (sceneZMax != 0.0); 
		 P += dP, Q.z += dQ.z, k += dk, ++stepCount) 
	{
		 
		hitPixel = permute ? P.yx : P.xy;
		
		// The depth range that the ray covers within this loop
		// iteration.  Assume that the ray is moving in increasing z
		// and swap if backwards.  Because one end of the interval is
		// shared between adjacent iterations, we track the previous
		// value and then swap as needed to ensure correct ordering
		rayZMin = prevZMaxEstimate;
		
		
		// Compute the value at 1/2 pixel into the future
		//if it is is smaller (distance = negative) then swap them for the next step
		rayZMax = (dQ.z * 0.5 + Q.z) / (dk * 0.5 + k); //F: here we re-homgenize so we are in actual view space again (?)
		prevZMaxEstimate = rayZMax;
		if (rayZMin > rayZMax) { 
			//swap
			float t = rayZMin; 
			rayZMin = rayZMax; 
			rayZMax = t;
		}
 
		
		// You may need hitPixel.y = csZBufferSize.y - hitPixel.y; here if your vertical axis is different than ours in screen space
		//hitPixel.y = csZBufferSize.y - hitPixel.y;
		// Camera-space z of the background
		//vec4 cameraSpacePos = viewMatrix *  vec4(texelFetch(uTexGbuffer_WorldPos, ivec2(hitPixel), 0).xyz,1.0); //!!!negative z-buffer expected --> value at FAR = -1.0
		//sceneZMax = -cameraSpacePos.z;
		
		vec4 cameraSpacePos = viewMatrix * vec4(WorldPosFromDepth(envDomeDepth,hitPixel*resInv),1.0);
		sceneZMax = cameraSpacePos.z;
		
		//return cameraSpacePos;
		
		//sceneZMax = texture(uTexGbuffer_WorldPos, hitPixel*resInv).a;
		//sceneZMax = -texture(uTexGbuffer_WorldPos, hitPixel).z;
		
	}
	
	float rayhit = !((rayZMax >= sceneZMax - zThickness) && (rayZMin < sceneZMax)) ? 0.0 : 1.0;
	
	// Advance Q based on the number of steps
	// this is done once after the loop instead of incrementing it inside the loop
	// the z value is incremented inside the loop for checking the difference
	Q.xy += dQ.xy * stepCount;
	hitPoint = Q * (1.0 / k); //revert the mapping to NEAR to get the 3d position of the point hit, k is being incremented inside the loop
	
	//return vec4(hitPoint,1.0);
	
	vec2 screenUV = gl_FragCoord.xy*resInv;

	vec2 hitUV = hitPixel*resInv;
	
	vec3 SSR = texture(envDome, hitUV).rgb;
	float SSR_lum = SSR.r * .3 * SSR.g * .6 * SSR.b * .1;
	
	vec3 fallback = vec3(0.0,1.0,1.0);
	/*
	#ifdef WITH_PARALLAX_CORRECTION
		fallback  = getEnvmap_Static_ParallaxCorrection(posW, view, normal, uBB_min, uBB_max, uBB_center);
	#else
		fallback = getEnvmap_Static(normalize(reflect(view,normal)));
	#endif
	*/
	
	float ray_segment_behind = (rayZMax - (sceneZMax - zThickness));
	float ray_segment_infront = (rayZMin - sceneZMax);
	float ray_segment = abs((ray_segment_behind - ray_segment_infront))/350.0;
	
	float distanceFade = 1.0-(-rayZMin)/maxDistance;
	float stepFade = 1.0-stepCount/maxSteps;
	
	float fade1 = distanceFade*stepFade*3.0;
	
	vec2 vignetteUV = hitUV * (1.0 - hitUV.yx);//hitUV! if whatever was hit was close to the screens border fade it
	float vig = vignetteUV.x*vignetteUV.y*50.0;
	vig = clamp(vig,0.0,1.0);
	
	float blend = clamp((rayhit*vig*fade1),0.0,1.0);
	
	vec3 finalColor = vec3(0.0);
	//SSR*= SSR*1.5;
	//fallback*=fallback*0.25;
	//fallback+= 0.025;
	finalColor = mix(fallback,SSR,blend);
	//finalColor = clamp(finalColor,0.0,1.0);
	return vec4(SSR,blend);
	
}

void main() 
{ 
	//re-normalize after interpolation, else you got extremely visible triange artifacts!!!
	vec3 N = normalize(Normal);
	vec3 L = normalize(Light);
	//vec3 V = normalize(View);
	vec3 V = normalize(cameraPosition-posW);
	vec3 H = normalize(Halfway);
	//vec3 H = normalize(vec3(L + V));

	vec3 specColor = mix(vec3(1.0),albedoColor,metalness);

	float depth = texture(envDomeDepth,gl_FragCoord.xy*vec2(1.0/1024.0,1.0/768.0)).x*0.1;
	vec3 pos_from_depth = WorldPosFromDepth(envDomeDepth,gl_FragCoord.xy*vec2(1.0/1024.0,1.0/768.0));
	
	vec2 screenUV = gl_FragCoord.xy*vec2(1.0/1024.0,1.0/768.0);
	
	vec3 backNormal = texture(backNormal,screenUV).xyz;
	
	vec4 ssr_mask = SSR(V, N);
	vec3 ssr = ssr_mask.rgb;
	float mask = ssr_mask.a;
	
	vec3 phong = albedoColor*(ambientColor + Phong_diffuse(N,L)*lightColor) + Phong_Specular(N,H)*specColor*lightColor;
	out0 = vec4((pos_from_depth)*0.1,1.0);
	//out0 = vec4((posW)*0.1,1.0);
	out0 = vec4(mix(phong,ssr,mask*0.25),1.0);
	//out0 = vec4(ssr,1.0);
	//out0 = vec4(vec3(abs(depth)),1.0);
}