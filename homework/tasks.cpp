#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        // vec3 o = ray.Origin, dir = ray.Direction;
        glm::mat3 M = glm::mat3(p1 - p2, p1 - p3, ray.Direction);
        glm::vec3 Y = p1 - ray.Origin;
        if (glm::abs(glm::determinant(M)) < 1e-12) { 
            return false;
        }
        glm::vec3 T = glm::inverse(M) * Y;
        if (T.z < 1e-12 || T.x < 0 || T.y < 0 || T.x + T.y > 1) { 
            return false;
        }
        output.t = T.z;
        output.u = T.x;
        output.v = T.y;
        return true;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);
        // return glm::vec3(1.0f,0.0f,0.0f);
        for (int depth = 0; depth < maxDepth; depth++) {
            // color += glm::vec3(0.1f,0.0f,0.0f);
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;
            const glm::vec3 n         = rayHit.IntersectNormal;
            const glm::vec3 kd        = rayHit.IntersectAlbedo;
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;
            const float     alpha     = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here

            glm::vec3 N = glm::normalize(n);
            glm::vec3 V = glm::normalize(-ray.Direction);


            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        // float dist_0 = glm::length(l);
                        glm::vec3 light_dir = glm::normalize(l);

                        bool inShadow = false;
                        glm::vec3 origin = pos + (1e-4f) * N;
                        for (int q = 0; q < 1e7; ++q) {
                            float dist_0 = glm::length(light.Position - origin);
                            Ray shadow_ray(origin, light_dir);
                            auto hit = intersector.IntersectRay(shadow_ray);
                            if (!hit.IntersectState) {
                                // no other obstacle
                                break;
                            }

                            float dist_hit = glm::length(hit.IntersectPosition - pos);

                            if (dist_hit > dist_0 + 1e-7) {
                                break;  // already to far
                            }

                            // alpha < 0.2f  transparent
                            float hitAlpha = hit.IntersectAlbedo.w;
                            if (hitAlpha >= 0.2f) {
                                inShadow = true;
                                break;
                            }
                            origin = hit.IntersectPosition + light_dir * 0.001f;
                        }

                        if (inShadow) {
                            continue;
                        }
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        // float dist_0 = glm::length(l);
                        glm::vec3 light_dir = glm::normalize(l);

                        bool inShadow = false;
                        glm::vec3 origin = pos + (1e-4f) * N;
                        for (int q = 0; q < 1e7; ++q) {
                            Ray shadow_ray(origin, light_dir);
                            auto hit = intersector.IntersectRay(shadow_ray);
                            if (!hit.IntersectState) {
                                // no other obstacle
                                break;
                            }

                            float dist_hit = glm::length(hit.IntersectPosition - pos);

                            // alpha < 0.2f  transparent
                            float hitAlpha = hit.IntersectAlbedo.w;
                            if (hitAlpha >= 0.2f) {
                                inShadow = true;
                                break;
                            }
                            origin = hit.IntersectPosition + light_dir * 0.001f;
                        }

                        if (inShadow) {
                            continue;
                        }
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                if (attenuation > 0.0f){
                    glm::vec3 lightDir = glm::normalize(l);
                    
                    float NdotL = glm::max(glm::dot(N, lightDir), 0.0f);
                    if (NdotL > 0.0f) {
                        glm::vec3 diffuse = kd * light.Intensity * NdotL;
                        
                        // Blinn-Phong
                        glm::vec3 viewDir = glm::normalize(-ray.Direction);
                        glm::vec3 halfDir = glm::normalize(lightDir + viewDir);
                        float NdotH = glm::max(glm::dot(N, halfDir), 0.0f);
                        glm::vec3 specular = ks * light.Intensity * glm::pow(NdotH, shininess);
                        
                        result += (diffuse + specular) * attenuation;
                    }
                }
            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering