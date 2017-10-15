#version 450

in vec3 vNormalMVP;

uniform bool derivative;

layout(location = 0) out vec3 outColor;

void main()
{       

	float moment1 = gl_FragCoord.z; //moment1 is stored in the regular depth already!
	float moment2 = moment1*moment1; //moment2 is stored in R
	float moment3 = moment2*moment1; //moment2 is stored in G	
	float moment4 = moment2*moment2; //moment2 is stored in B
	
	
//one ove the VSM implementations stores this here, not sure if correct. Do here or after filtering?
if(derivative){
	//derivates for bias to prevent shadow-acne
	float dx = dFdx(moment1);
	float dy = dFdy(moment1);
	moment2 += 0.25*(dx*dx+dy*dy);
}
	
	outColor = vec3(moment2, moment3, moment4);
}  