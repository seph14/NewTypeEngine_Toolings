#version 330 core

in vec3 vNormal;
in vec3 vViewDir;
in vec2 vTexcoord;

out vec4 oColor;

const vec3 _lightDir = vec3(.0,-1.,.0);

#define saturate(x) clamp(x, 0.0000001, 1.0)

float luma(vec3 color) {
  return dot(color, vec3(0.299, 0.587, 0.114));
}

float wetSpecular(vec3 n,vec3 l,vec3 e,float s) {    
    float nrm = (s + 8.0) / (3.1415 * 8.0);
    return pow(max(dot(reflect(e,n),l),0.0),s) * nrm;
}

float getNormalDistribution( float roughness4, float NoH ){
  float d = ( NoH * roughness4 - NoH ) * NoH + 1.0;
  return roughness4 / ( d*d );
}

// Smith GGX geometric shadowing
float getGeometricShadowing( float roughness4, float NoV, float NoL, float VoH, vec3 L, vec3 V ){ 
  float gSmithV = NoV + sqrt( NoV * (NoV - NoV * roughness4) + roughness4 );
  float gSmithL = NoL + sqrt( NoL * (NoL - NoL * roughness4) + roughness4 );
  return 1.0 / ( gSmithV * gSmithL );
}

const lowp float A = 0.15;
const lowp float B = 0.50;
const lowp float C = 0.10;
const lowp float D = 0.20;
const lowp float E = 0.02;
const lowp float F = 0.30;
lowp vec3 Uncharted2Tonemap( lowp vec3 x ){
   return ((x*(A*x+C*B)+D*E)/(x*(A*x+B)+D*F))-E/F;
}

const float W = 11.2;
vec3 filmicMap(vec3 col){
    const float ExposureBias = 2.0;
    vec3 curr       = ExposureBias * Uncharted2Tonemap(col);
    vec3 whiteScale = 1.0/Uncharted2Tonemap(vec3(W));
    vec3 color      = curr*whiteScale;
    vec3 retColor   = pow(color, vec3(1.0/2.2));
    return retColor;
}

// for plants
// https://twvideo01.ubm-us.net/o1/vault/gdc2017/Presentations/Hammon_Earl_PBR_Diffuse_Lighting.pdf
vec3 getDiffuse(vec3 L, vec3 V, vec3 N, vec3 albedo, float alpha){
	float LpV 	= length(L + V);
  	float NoL   = dot( N, L );
  	float NoV   = dot( N, V );
  	float NoH   = (NoL + NoV) / LpV; 

  	float LoH   = 0.5 * LpV;
  	float mNoL  = 1.0 - NoL;
  	float mNoV  = 1.0 - NoV;

  	float facing = LpV * LpV / 4.0;
  	float rough  = facing * (0.9 - 0.4 * facing) * (0.5 + NoH) / NoH;
  	float smoothn= 1.05*(1.0-pow(mNoL,5.0))*(1.0-pow(mNoV,5.0));
  	float single = mix(smoothn, rough, alpha) / 3.14159265358;
  	float multi  = 0.1159 * alpha;

  	//return vec3(luma(texture(uRampMap, vec2(single, .5)).rgb)); //vec3(1. - single);

  	return albedo * (single + albedo * multi);
}

#define PI 3.14159265359

vec3 ImportanceSampleGGX( vec2 Xi, float Roughness, vec3 N ){
    float a = Roughness * Roughness;
    float Phi = 2.0 * PI * Xi.x;
    float CosTheta = sqrt( (1.0 - Xi.y) / ( 1.0 + (a*a - 1.0) * Xi.y ) );
    float SinTheta = sqrt( 1.0 - CosTheta * CosTheta );
    vec3 H;
    H.x = SinTheta * cos( Phi );
    H.y = SinTheta * sin( Phi );
    H.z = CosTheta;
    vec3 UpVector = abs(N.z) < 0.999 ? vec3(0,0,1) : vec3(1,0,0);
    vec3 TangentX = normalize( cross( UpVector, N ) );
    vec3 TangentY = cross( N, TangentX );
    // Tangent to world space
    return TangentX * H.x + TangentY * H.y + N * H.z;
}

//micro shadowing for details
//http://advances.realtimerendering.com/other/2016/naughty_dog/NaughtyDog_TechArt_Final.pdf
float ApplyMicroShadow(float ao, vec3 N, vec3 L){
	float aperture = 2.0 * ao * ao;
	float microShadow = saturate(abs(dot(L,N)) + aperture - 1.0);
	return microShadow;
}

//Ambient Occlusion Fresnel
float getAOOcclusion(float ao, vec3 N, vec3 V){
	float aoFadeTerm = saturate(dot(N,V));
	return mix(1.0,ao,aoFadeTerm);
}

//light wrap for micro fiber surfaces
float ApplyLightWrap(float lightWrapColor, vec3 normalWS, vec3 vertexNormalWS, vec3 lightDirWS){
	float lightWrapDisance = 1.0;
	vec3 wrapLight = vec3(lightWrapDisance * lightWrapColor);
	float NdotL = dot(normalWS, lightDirWS);
	NdotL = mix(max(wrapLight.r,max(wrapLight.g,wrapLight.b)), 1.0, NdotL);
	float wrapForwardNdotL = max(NdotL, dot(vertexNormalWS, lightDirWS));
	vec3 wrapForward = mix(wrapLight, vec3(1.0), wrapForwardNdotL);
	vec3 wrapRecede  = mix(-wrapLight,vec3(1.0),NdotL);
	vec3 wrapLighting = saturate(mix(wrapRecede, wrapForward, vec3(1. - lightWrapColor)));
	return luma(wrapLighting);
}

// general diffuse
// http://blog.selfshadow.com/publications/s2016-shading-course/hoffman/s2016_pbs_recent_advances_v2.pdf
vec3 getDiffuseBrentBurley(vec3 L, vec3 V, vec3 N, vec3 albedo, float alpha){
    float LpV   = length(L + V);
    float NoL   = dot( N, L );
    float NoV   = dot( N, V );
    float NoH   = (NoL + NoV) / LpV; 

    float LoH    = 0.5 * LpV;
    float Fl     = pow(1.0 - NoL,0.5);
    float Fv     = pow(1.0 - NoV,0.5);
    float Rr     = 2 * alpha * NoH * NoH;
    vec3 lambert = albedo / PI;
    float ratio  = (1.0-0.5*Fl)*(1.0-0.5*Fv) + Rr*(Fl+Fv+Fl*Fv*(Rr-1.0));
    return lambert * ratio; 
}

vec3 blendNormals( vec3 baseNormal, vec3 detailsNormal ){
    vec3 n1 = baseNormal;
    vec3 n2 = detailsNormal;
    mat3 nBasis = mat3(
        vec3(n1.z, n1.y, -n1.x), 
        vec3(n1.x, n1.z, -n1.y), 
        vec3(n1.x, n1.y,  n1.z));
    return normalize(n2.x*nBasis[0] + n2.y*nBasis[1] + n2.z*nBasis[2]);
}

vec3 udnBlend( vec3 baseNormal, vec3 detailsNormal ){
  return normalize(vec3(baseNormal.xy + detailsNormal.xy, baseNormal.z));
}

float PseudoRandom(vec2 xy){
    // found by experimentation
    vec2 pos = fract(xy / 128.0) * 128.0 + vec2(-64.340622, -72.465622);
    return fract(dot(pos.xyx * pos.xyy, vec3(20.390625, 60.703125, 2.4281209)));
}

void main( void ){
	oColor.a 		= 1.;
	
	vec3 V 			= normalize(vViewDir);
	vec3 N 			= normalize(vNormal);
	vec3 lookup 	= reflect( V, N );
	
	vec3 diffuse 	= getDiffuseBrentBurley(_lightDir, V, N, vec3(.8), 1.5);//getDiffuse( lightDir, V, N, albedo, .4);
	float shadow 	= getGeometricShadowing( 1., dot(N,V), dot(N,_lightDir), dot(V, normalize(V+_lightDir)), _lightDir, V );
	float wet 		= wetSpecular(N, _lightDir, V, 64.);
	float wrap  	= saturate(wet + shadow * pow(diffuse.x, 4.0));
	
	oColor.rgb  = vec3( vTexcoord, wrap );
	oColor.a 	= 1.;
}
