/* ************************************************************************
	Rules:
    1. The universe is a two-dimensional grid of 8-connected square cells.
    2. A cell is either dead or alive.
    3. A dead cell with exactly three living neighbors comes to life.
    4. A live cell with less than two neighbors dies from underpopulation.
    5. A live cell with more than three neighbors dies from overpopulation.
************************************************************************ */

#version 430

#define gid gl_GlobalInvocationID.xy				// = gl_WorkGroupID * gl_WorkGroupSize + gl_LocalInvocationID. 

layout(local_size_x = 32, local_size_y = 32) in;

layout(binding=0, rgba32f) uniform image2D outputValue;
	
uniform ivec2 res;
uniform int type;

/* ************************************************************************
	0. Random Number
	Returns a random number between [0,1[
************************************************************************ */

float randomWithSeedsSin (vec2 st, float seed1, float seed2, float seed3) {
    return fract(sin(dot(st.xy, vec2(seed1, seed2))) * seed3);
}


vec3 simpleColorNoise(vec2 st){
	float randomX = round( randomWithSeedsSin(st, 12.9898, 78.233, 43758.5453123));
	float randomY = round( randomWithSeedsSin(st, 3.4962324, 7.34561, 84638.34566876));
	float randomZ = round( randomWithSeedsSin(st, 67.02383, 54.34523, 190573.31345345));
    return vec3(randomX, randomY, randomZ); 
} 


/* ************************************************************************
	MAIN
************************************************************************ */
void main() {
		///////////////////////////////////////////////////////////////
		// One quad (for bugged scene)
		///////////////////////////////////////////////////////////////
		if(type == 0){
			vec4 color = vec4(0.0, 0.0, 0.0, 1.0);

			if(gid.x == 15 && gid.y == 15){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}

			if(gid.x == 16 && gid.y == 15){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}

			if(gid.x == 15 && gid.y == 16){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}

			if(gid.x == 16 && gid.y == 16){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}

			imageStore(outputValue, ivec2(gid), color);
		}

		///////////////////////////////////////////////////////////////
		// Random points
		///////////////////////////////////////////////////////////////
		if(type == 1){
			vec4 color = vec4(1.0, 1.0, 1.0, 1.0);
			vec2 relativePixelPos = vec2(float(gid.x)/float(res.x), float(gid.y)/float(res.y));

			float reduceRandom = 0.0;
			float randomNumber = clamp(randomWithSeedsSin(relativePixelPos, 12.9898, 78.233, 43758.5453123) - reduceRandom, 0.0, 1.0);
			color = vec4(round(randomNumber), 0.0, 0.0, 1.0);

			imageStore(outputValue, ivec2(gid), color);
		}
		
}


