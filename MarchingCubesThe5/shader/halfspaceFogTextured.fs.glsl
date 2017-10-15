#version 330
  
layout(location = 0)  out vec4 out0; // color 

uniform sampler2D texture_diffuse1;
uniform float k;            // (F dot C <= 0.0) 

in vec3 rhoV; 
in float F_dot_V; 
in float F_dot_P; 

in vec2 TexCoords;

void main() 
{ 
	vec4 unfoggedColor = texture(texture_diffuse1, TexCoords);

   // Calculate distance * rho using Equation (5) 
   float d = clamp(k - F_dot_P / abs(F_dot_V), 0.0, 1.0); 

   d *= length(rhoV); 
   // Calculate fog fraction and apply 

   float f = clamp(exp2(-d), 0.0, 1.0);

   vec3 glFogParameters_color = vec3(1.0, 1.0, 1.0);

   out0 = vec4(unfoggedColor.rgb * f + glFogParameters_color * (1.0 - f), 1.0); 
} 
