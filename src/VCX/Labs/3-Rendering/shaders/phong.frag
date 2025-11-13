#version 410 core

layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;
layout(location = 2) in  vec2 v_TexCoord;

layout(location = 0) out vec4 f_Color;

struct Light {
    vec3  Intensity;
    vec3  Direction;   // For spot and directional lights.
    vec3  Position;    // For point and spot lights.
    float CutOff;      // For spot lights.
    float OuterCutOff; // For spot lights.
};

layout(std140) uniform PassConstants {
    mat4  u_Projection;
    mat4  u_View;
    vec3  u_ViewPosition;
    vec3  u_AmbientIntensity;
    Light u_Lights[4];
    int   u_CntPointLights;
    int   u_CntSpotLights;
    int   u_CntDirectionalLights;
};

uniform float u_AmbientScale;
uniform bool  u_UseBlinn;
uniform float u_Shininess;
uniform bool  u_UseGammaCorrection;
uniform int   u_AttenuationOrder;
uniform float u_BumpMappingBlend;

uniform sampler2D u_DiffuseMap;
uniform sampler2D u_SpecularMap;
uniform sampler2D u_HeightMap;

vec3 Shade(vec3 lightIntensity, vec3 lightDir, vec3 normal, vec3 viewDir, vec3 diffuseColor, vec3 specularColor, float shininess) {
    // your code here:
    // lightDir = normalize(lightDir), viewDir = normalize(viewDir), normal = normalize(normal);
    float cos3 = (u_UseBlinn)? dot(normalize(viewDir + lightDir), normal) : dot(viewDir, normalize(2 * dot(normal, lightDir) * normal - lightDir));
    return lightIntensity * max(dot(lightDir, normal),0) * diffuseColor + lightIntensity * pow(max(cos3, 0), shininess) * specularColor;
}

vec3 GetNormal() {
    // Bump mapping from paper: Bump Mapping Unparametrized Surfaces on the GPU
    vec3 vn = normalize(v_Normal);
    // your code here:
    if (u_BumpMappingBlend > 0.0){
    //if (false){ }
        // use Bump mapping
        // orginal normal is v_Normal
        float height = texture(u_HeightMap, v_TexCoord).r;
        // HeightMap use TexCoord as coordinates
        float hDx = dFdx(height);
        float hDy = dFdy(height);
        // calculate on GPU ,compare with nearby pixels
        vec3 pos_dx = dFdx(v_Position);
        vec3 pos_dy = dFdy(v_Position);
        
        vec3 tangent = pos_dx + vn * hDx;
        vec3 bitangent = pos_dy + vn * hDy;
        vec3 bumpNormal = normalize(cross(tangent, bitangent));
        return normalize(mix(vn, bumpNormal, u_BumpMappingBlend));
    }
    else{
        return vn;
    }
}

void main() {
    float gamma          = 2.2;
    vec4  diffuseFactor  = texture(u_DiffuseMap , v_TexCoord).rgba;
    vec4  specularFactor = texture(u_SpecularMap, v_TexCoord).rgba;
    if (diffuseFactor.a < .2) discard;
    vec3  diffuseColor   = u_UseGammaCorrection ? pow(diffuseFactor.rgb, vec3(gamma)) : diffuseFactor.rgb;
    vec3  specularColor  = specularFactor.rgb;
    float shininess      = u_Shininess < 0 ? specularFactor.a * 256 : u_Shininess;
    vec3  normal         = GetNormal();
    vec3  viewDir        = normalize(u_ViewPosition - v_Position);
    // Ambient component.
    vec3  total = u_AmbientIntensity * u_AmbientScale * diffuseColor;
    // Iterate lights.
    for (int i = 0; i < u_CntPointLights; i++) {
        vec3  lightDir     = normalize(u_Lights[i].Position - v_Position);
        float dist         = length(u_Lights[i].Position - v_Position);
        float attenuation  = 1. / (u_AttenuationOrder == 2 ? dist * dist : (u_AttenuationOrder == 1  ? dist : 1.));
        total             += Shade(u_Lights[i].Intensity, lightDir, normal, viewDir, diffuseColor, specularColor, shininess) * attenuation;
    }
    for (int i = u_CntPointLights + u_CntSpotLights; i < u_CntPointLights + u_CntSpotLights + u_CntDirectionalLights; i++) {
        total += Shade(u_Lights[i].Intensity, u_Lights[i].Direction, normal, viewDir, diffuseColor, specularColor, shininess);
    }
    // Gamma correction.
    f_Color = vec4(u_UseGammaCorrection ? pow(total, vec3(1. / gamma)) : total, 1.);
}
