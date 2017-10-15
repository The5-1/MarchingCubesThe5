#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture_diffuse;
uniform sampler2D texture_specular;
uniform sampler2D texture_normal;
uniform sampler2D texture_height;

void main()
{ 

FragColor = texture(texture_diffuse, TexCoords);
   
/*
    albedo = texture(texture_diffuse, TexCoords);
    specular = texture(texture_specular, TexCoords);
    normal = texture(texture_normal, TexCoords);
    height = texture(texture_height, TexCoords);
*/
}

