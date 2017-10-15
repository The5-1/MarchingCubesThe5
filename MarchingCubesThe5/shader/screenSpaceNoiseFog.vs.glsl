//http://isnippets.blogspot.de/2010/10/real-time-fog-using-post-processing-in.html
//http://www.iquilezles.org/www/articles/fog/fog.htm
//https://blog.demofox.org/2014/06/22/analytic-fog-density/

#version 330

layout(location = 0) in  vec3 vPosition; 

out vec2 texCoords;

void main() {
	texCoords = vPosition.xy * vec2(0.5) + vec2(0.5);
	gl_Position = vec4(vPosition, 1.0);
}

