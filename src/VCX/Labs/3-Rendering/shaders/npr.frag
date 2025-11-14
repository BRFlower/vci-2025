#version 410 core

layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;

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

uniform vec3 u_CoolColor;
uniform vec3 u_WarmColor;

uniform float s1 = 0.6, s2 = -0.2;
uniform float s3 = 0.2; // average of s1,s2

vec3 Shade (vec3 lightDir, vec3 normal) {
    // your code here:
    float ans = dot(normal, lightDir);
    if (ans > s1) {
        return u_WarmColor;
    }
    else if (ans < s2){
        return u_CoolColor;
    }
    else{
        return (u_WarmColor * (1 - s3) + u_CoolColor * (1 + s3)) * 0.5;
    }
    //return vec3(0);
}

void main() {
    // your code here:
    float gamma = 2.2;
    vec3 total = Shade(u_Lights[0].Direction, v_Normal);
    f_Color = vec4(pow(total, vec3(1. / gamma)), 1.);
}
