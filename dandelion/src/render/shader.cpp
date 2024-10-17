#include "rasterizer_renderer.h"
#include "../utils/math.hpp"
#include <cstdio>

#ifdef _WIN32
#undef min
#undef max
#endif

using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;

    // Vertex position transformation
    output_payload.world_position = payload.world_position;
    // Viewport transformation
    Eigen::Matrix4f viewport_matrix;
    viewport_matrix << Uniforms::width / 2.0f, 0, 0, Uniforms::width / 2.0f,
        0, Uniforms::height / 2.0f, 0, Uniforms::height / 2.0f,
        0, 0, 0.5f, 0.5f,
        0, 0, 0, 1;
    output_payload.viewport_position = Uniforms::MVP *  payload.world_position;
    output_payload.viewport_position = viewport_matrix * (output_payload.viewport_position / output_payload.viewport_position.w());

    // Vertex normal transformation
    Vector4f normal_4d = {payload.normal[0], payload.normal[1], payload.normal[2], 0};
    output_payload.normal = (Uniforms::inv_trans_M * normal_4d).head(3).normalized();

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, const GL::Material& material,
                               const std::list<Light>& lights, const Camera& camera)
{
    Vector3f result = {0, 0, 0};

    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    auto ka = material.ambient;
    auto kd = material.diffuse;
    auto ks = material.specular;

    // set ambient light intensity
    float ambient_intensity = 0.3f; 

    // Ambient
    result += ka * ambient_intensity; 

    for(const auto& light : lights)
    {
        // Light Direction
        auto light_direction = (light.position - payload.world_pos).normalized();

        // View Direction
        auto view_direction = (camera.position - payload.world_pos).normalized();

        // Half Vector
        auto half_vector = (light_direction + view_direction).normalized();

        // Light Attenuation
        auto distance_fragment_light = (light.position - payload.world_pos).norm();
        auto light_attenuation = 1 / (distance_fragment_light * distance_fragment_light);
        auto _normal = payload.world_normal.normalized();
        // Diffuse
        result += light.intensity * light_attenuation * kd * std::max(0.0f, _normal.dot(light_direction));
        // Specular
        result += light.intensity * light_attenuation * ks * std::pow(std::max(0.0f, _normal.dot(half_vector)), material.shininess);
    }

    // set rendering result max threshold to 255
    if(result.x() > 1.0f) { result.x() = 1.0f; }
    if(result.y() > 1.0f) { result.y() = 1.0f; }
    if(result.z() > 1.0f) { result.z() = 1.0f; }
    
    return result * 255.f;
}
