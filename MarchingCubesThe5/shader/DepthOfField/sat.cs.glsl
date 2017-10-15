#version 430
#
// TODO: Read the OpenGL 4.3 and GLSL 430 documentation on compute shaders. At
// least you should have a look at texture samplers versus image textures, as
// well as on the work group configuration and gl_GlobalInvocationID.

#define gid gl_GlobalInvocationID.xy
layout(local_size_x = 32, local_size_y = 32) in;
layout(binding=0) uniform sampler2D inputData;
layout(binding=1, rgba32f) uniform image2D outputValue;
	
// TODO: extend list of uniforms.
uniform ivec2 res;
uniform int pass;
uniform int stepstuff;

void main() {
	// TODO: border handling
		vec4 lastVal = texelFetch(inputData, ivec2(gid), 0);
		vec4 nextVal = vec4(0);
		
		ivec2 offs;
		if(stepstuff == 0)
            offs = ivec2(gid.x + pow(2, pass), gid.y);
        else
            offs = ivec2(gid.x, gid.y + pow(2, pass));
            
        if(offs.x < res.x && offs.y < res.y)
            nextVal = texelFetch(inputData, offs, 0);
		

		imageStore(outputValue, ivec2(gid), lastVal + nextVal);

		//imageStore(outputValue, ivec2(gid), vec4(1.0, 0.0, 0.0, 1.0));
}

