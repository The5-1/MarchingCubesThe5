#version 330
  
layout(location = 0)  out vec4 out0; // color 

in vec2 tc;

uniform sampler2D sat;
uniform sampler2D color;
uniform sampler2D depth;

uniform bool manualBlur;
uniform bool debugSat;
uniform float blur;
uniform float focalPlane;
uniform ivec2 res;


vec4 sat_lookup(sampler2D tex, ivec2 min, ivec2 max) {
	// TODO: implement lookup.
	// NOTE: handle border cases.
	// NOTE: the area bounds are understood to be inclusive.
	max = ivec2(clamp(max.x, 0, res.x-1), clamp(max.y, 0, res.y-1));
	min = ivec2(clamp(min.x, 0, res.x-1), clamp(min.y, 0, res.y-1));
	return (texelFetch(tex, min, 0)
            + texelFetch(tex, max + ivec2(1, 1), 0)
            - texelFetch(tex, min + ivec2(0, max.y-min.y+1), 0)
            - texelFetch(tex, min + ivec2(max.x-min.x+1, 0), 0))/((max.x-min.x+1)*(max.y-min.y+1));
}

void main() {
	int r = 0;

	if (manualBlur)
		r = int(blur);
	else {
		float linear_depth = texture(color, tc).a;
		//float linear_depth = texture(depth, tc).r;

		float coc = 0;

		// TODO: compute circle of confusion.
		coc = 10*abs(linear_depth - focalPlane)/linear_depth;
		r = min(int(coc), 40);
	}

	vec3 col = sat_lookup(sat, ivec2(gl_FragCoord.xy - vec2(r,r)), ivec2(gl_FragCoord.xy + vec2(r,r))).rgb;
	
	out0 = vec4(col, 1);
	
	gl_FragDepth = texture(depth, tc).r;
}
