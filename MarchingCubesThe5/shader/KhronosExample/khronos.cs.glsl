#version 430 compatibility
#extension GL_ARB_compute_shader :      enable
#extension GL_ARB_shader_storage_buffer_object :   enable;

layout( std140, binding=4 )  buffer  Pos 
{
vec4  Positions[   ];		// array of structures
};

layout( std140, binding=5 )  buffer   Vel
{
vec4  Velocities[   ];		// array of structures
};

layout( local_size_x = 128,  local_size_y = 1, local_size_z = 1 )   in;

void main(){
	const  vec3  G        =  vec3( 0., -9.8, 0. );
	const  float   DT      =  0.1;

	uint gid = gl_GlobalInvocationID.x;						//y and z are both 1 in this case

	vec3 p  = Positions[  gid  ].xyz;
	vec3 v  = Velocities[  gid  ].xyz;
	vec3  pp = p + v*DT + .5*DT*DT*G;
	vec3  vp = v + G*DT;
	Positions[  gid  ].xyz  = pp;
	Velocities[  gid  ].xyz = vp;
}