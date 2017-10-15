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

layout(binding=0, rgba32f) uniform image2D inputValue;
layout(binding=1, rgba32f) uniform image2D outputValue;
	
uniform ivec2 res;
uniform int type;

/*
void copyImage(){
	vec4 color = imageLoad(inputValue, ivec2(gid));
	imageStore(outputValue, ivec2(gid), color);
}
*/

/* ************************************************************************
MAIN

Good to know for imageLoad and imageStor: https://www.khronos.org/opengl/wiki/Image_Load_Store
-Load operations from any texel that is outside of the boundaries of the bound image will return all zeros.
************************************************************************ */

void main() {

		vec4 color = vec4(0.0, 0.0, 0.0, 1.0);

		bool alive; 
		if(imageLoad(inputValue, ivec2(gid)).r == 1.0){
			alive = true;
		}
		else{
			alive = false;
		}

		int neighbours = 0;
		
		if(imageLoad(inputValue, ivec2(gid + vec2(0, 1))).r == 1.0){
			neighbours++;
		}

		if(imageLoad(inputValue, ivec2(gid + vec2(0, -1))).r == 1.0){
			neighbours++;
		}

		if(imageLoad(inputValue, ivec2(gid + vec2(1, 0))).r == 1.0){
			neighbours++;
		}
		if(imageLoad(inputValue, ivec2(gid + vec2(-1, 0))).r == 1.0){
			neighbours++;
		}

		if(imageLoad(inputValue, ivec2(gid + vec2(1, 1))).r == 1.0){
			neighbours++;
		}

		if(imageLoad(inputValue, ivec2(gid + vec2(1, -1))).r == 1.0){
			neighbours++;
		}

		if(imageLoad(inputValue, ivec2(gid + vec2(-1, -1))).r == 1.0){
			neighbours++;
		}

		if(imageLoad(inputValue, ivec2(gid + vec2(-1, 1))).r == 1.0){
			neighbours++;
		}
		
		///////////////////////////////////////////////////////////////
		// Interesting bugged Game of life
		///////////////////////////////////////////////////////////////
		if(type == 0){
			if(neighbours < 2){
				color = vec4(0.0, 0.0, 0.0, 1.0);
			}
			else if(neighbours == 2 || neighbours == 3){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}
			else{
				color = vec4(0.0, 0.0, 0.0, 1.0);
			}
		}

		///////////////////////////////////////////////////////////////
		// Game of life
		///////////////////////////////////////////////////////////////
		if(type == 1){
			if(neighbours < 2){
				color = vec4(0.0, 0.0, 0.0, 1.0);
			}
			else if(!alive && neighbours == 3){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}
			else if(neighbours > 3){
				color = vec4(0.0, 0.0, 0.0, 1.0);
			}
			else if((alive && neighbours == 3) || (alive && neighbours == 2)){
				color = vec4(1.0, 0.0, 0.0, 1.0);
			}
			else{
				color = vec4(0.0, 0.0, 0.0, 1.0);
			}
		}

		imageStore(outputValue, ivec2(gid), color);
		
}

