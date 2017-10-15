#version 330
  
layout(location = 0)  out vec4 out0; // color 

uniform sampler2D texture_diffuse1;
uniform float k;            // (F dot C <= 0.0) 

in float c1; // k * (F dot P + F dot C)
in float c2; // (1 - 2k) * (F dot P)
in float F_dot_V;
in vec3 aV;
in vec2 TexCoords;

void main() 
{ 
   //exponentail halfspace fog texture
   vec4 color = texture(texture_diffuse1, TexCoords); // final color

	// Calculate g(P) using Equation (13)
	float g = min(c2, 0.0);

	g = -length(aV) * (c1 - g * g / abs(F_dot_V));

	// Calculate fog fraction and apply
	float f = clamp(exp2(-g), 0.0, 1.0);

	//Old OpenGL
	vec3 glFogParameters_color = vec3(1.0, 1.0, 1.0);

	out0 = vec4(color.rgb * f + glFogParameters_color * (1.0 - f), 1.0);
} 
