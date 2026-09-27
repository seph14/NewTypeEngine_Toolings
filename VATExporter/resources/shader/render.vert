#version 330 core

uniform float 	ciElapsedSeconds;
uniform mat4  	ciModelViewProjection;
uniform mat4  	ciModelView;
uniform mat3  	ciNormalMatrix;
uniform mat4 	ciModelMatrix;

layout(location = 1) in vec2 ciTexcoord;

out vec3 vNormal;
out vec3 vViewDir;
out vec2 vTexcoord;

uniform vec3 	uCamPos;
uniform vec3  	uSize, uBoundSize;
uniform int 	uVertCnt, uSide;
uniform float   uFrameCnt, uTime;

uniform samplerBuffer uNormalBuffer;
uniform samplerBuffer uVertexBuffer;

float max01(float val, float min, float max){
	return (val - min) / (max - min);
}

mat3 rotationMatrix(vec3 axis, float angle){
    axis     = normalize(axis);
    float s  = sin(angle);
    float c  = cos(angle);
    float oc = 1.0 - c;
    return mat3(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c 		   );
}

vec3 sampleAnimation( samplerBuffer buffer, out vec3 normal ){
	float frame 	= fract(uTime / uFrameCnt) * uFrameCnt;
	float ratio 	= fract(frame);
	int   beginFrm  = int(floor(mod( floor(frame) + 0., uFrameCnt)));
	int   endFrm 	= int(floor(mod( floor(frame) + 1., uFrameCnt)));

	vec3 pos0   	= texelFetch(uVertexBuffer, uVertCnt * beginFrm + gl_VertexID).xyz;
	vec3 pos1   	= texelFetch(uVertexBuffer, uVertCnt * endFrm   + gl_VertexID).xyz;

	vec3 norm0   	= texelFetch(uNormalBuffer, uVertCnt * beginFrm + gl_VertexID).xyz;
	vec3 norm1   	= texelFetch(uNormalBuffer, uVertCnt * endFrm   + gl_VertexID).xyz;
	normal 			= normalize(mix( norm0, norm1, ratio ));

	return mix( pos0, pos1, ratio );
}

void main( void ){
	float xx 		= float(gl_InstanceID % uSide) - .5 * uSide;
	float zz 		= float(gl_InstanceID / uSide) - .5 * uSide;

	vec4 pos 		= vec4( vec3(xx,0.,zz) * uBoundSize + uSize * sampleAnimation(uVertexBuffer, vNormal), 1. );
	vec3 worldpos 	= vec3(ciModelMatrix * pos);
	
	gl_Position	= ciModelViewProjection * pos;
	vTexcoord   = ciTexcoord;
	vNormal		= normalize((ciModelMatrix * vec4(vNormal,1.)).xyz);
	vViewDir 	= normalize(worldpos - uCamPos);
}
