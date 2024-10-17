#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <vector>
#include <optional>
#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "render_engine.h"
#include "../scene/light.h"
#include "../utils/math.hpp"
#include "../utils/ray.h"
#include "../utils/logger.h"

using std::chrono::steady_clock;
using duration   = std::chrono::duration<float>;
using time_point = std::chrono::time_point<steady_clock, duration>;
using Eigen::Vector3f;

// 最大的反射次数
constexpr int MAX_DEPTH        = 5;
constexpr float INFINITY_FLOAT = std::numeric_limits<float>::max();
// 考虑物体与光线相交点的偏移值
constexpr float EPSILON = 0.00001f;

// 当前物体的材质类型，根据不同材质类型光线会有不同的反射情况
enum class MaterialType
{
    DIFFUSE_AND_GLOSSY,
    REFLECTION
};

// 显示渲染的进度条
void update_progress(float progress)
{
    int barwidth = 70;
    std::cout << "[";
    int pos = static_cast<int>(barwidth * progress);
    for (int i = 0; i < barwidth; i++) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "]" << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

WhittedRenderer::WhittedRenderer(RenderEngine& engine)
    : width(engine.width), height(engine.height), n_threads(engine.n_threads), use_bvh(false),
      rendering_res(engine.rendering_res)
{
    logger = get_logger("Whitted Renderer");
}

// whitted-style渲染的实现
void WhittedRenderer::render(Scene& scene)
{
    time_point begin_time = steady_clock::now();
    width                 = std::floor(width);
    height                = std::floor(height);

    // initialize frame buffer
    std::vector<Vector3f> framebuffer(static_cast<size_t>(width * height));
    for (auto& v : framebuffer) {
        v = Vector3f(0.0f, 0.0f, 0.0f);
    }

    int idx = 0;
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            // generate ray
            Ray ray = generate_ray(static_cast<int>(width), static_cast<int>(height), i, j,
                                   scene.camera, 1.0f);
            // cast ray
            framebuffer[idx++] = cast_ray(ray, scene, 0);
        }
        update_progress(j / height);
    }
    static unsigned char color_res[3];
    rendering_res.clear();
    for (long unsigned int i = 0; i < framebuffer.size(); i++) {
        color_res[0] = static_cast<unsigned char>(255 * clamp(0.f, 1.f, framebuffer[i][0]));
        color_res[1] = static_cast<unsigned char>(255 * clamp(0.f, 1.f, framebuffer[i][1]));
        color_res[2] = static_cast<unsigned char>(255 * clamp(0.f, 1.f, framebuffer[i][2]));
        rendering_res.push_back(color_res[0]);
        rendering_res.push_back(color_res[1]);
        rendering_res.push_back(color_res[2]);
    }
    time_point end_time         = steady_clock::now();
    duration rendering_duration = end_time - begin_time;
    logger->info("rendering takes {:.6f} seconds", rendering_duration.count());
}

// 菲涅尔定理计算反射光线
float WhittedRenderer::fresnel(const Vector3f& I, const Vector3f& N, const float& ior)
{
    float cosi = clamp(-1.0f, 1.0f, I.dot(N));
    float etai = 1.0f, etat = ior;
    if (cosi > 0) {
        std::swap(etai, etat);
    }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.0f, 1.0f - cosi * cosi));
    // Total internal reflection
    if (sint >= 1.0f) {
        return 1.0f;
    } else {
        float cost = sqrtf(std::max(0.0f, 1.0f - sint * sint));
        cosi       = fabsf(cosi);
        float Rs   = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp   = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2.0f;
    }
}

// 如果相交返回Intersection结构体，如果不相交则返回false
std::optional<std::tuple<Intersection, GL::Material>> WhittedRenderer::trace(const Ray& ray,
                                                                             const Scene& scene)
{
    std::optional<Intersection> payload;
    Eigen::Matrix4f M;
    GL::Material material;
    for (const auto& group : scene.groups) {
        for (const auto& object : group->objects) {
            M = object->model();
            std::optional<Intersection> result;
            if (use_bvh) {
                result = object->bvh->intersect(ray, object->mesh, M);
            } else {
                result = naive_intersect(ray, object->mesh, M);
            }
            if (result.has_value() && (!payload.has_value() || result->t < payload->t)) {
                payload = result;
                material = object->mesh.material;
            }
        }
    }

    if (!payload.has_value()) {
        return std::nullopt;
    }
    return std::make_tuple(payload.value(), material);
}

// Whitted-style的光线传播算法实现
Vector3f WhittedRenderer::cast_ray(const Ray& ray, const Scene& scene, int depth)
{
    if (depth > MAX_DEPTH) {
        return Vector3f(0.0f, 0.0f, 0.0f);
    }
    // initialize hit color
    Vector3f hitcolor = RenderEngine::background_color;
    // get the result of trace()
    auto result = trace(ray, scene);

    // if result.has_value():
    // 1.judge the material_type
    // 2.if REFLECTION:
    //(1)use fresnel() to get kr
    //(2)hitcolor = cast_ray*kr
    // if DIFFUSE_AND_GLOSSY:
    //(1)compute shadow result using trace()
    //(2)hitcolor = diffuse*kd + specular*ks

    auto _ior_ = 1.65f;

    if(result.has_value()){
        auto [intersection, material] = result.value();
        Vector3f hit_point = ray.origin + ray.direction * intersection.t;
        
        MaterialType this_material_type = (material.shininess < 1000)? MaterialType::DIFFUSE_AND_GLOSSY : MaterialType::REFLECTION;
        switch(this_material_type){
            case MaterialType::REFLECTION:{
                // 计算反射光线
                Vector3f N = intersection.normal.normalized();
                Vector3f reflect_direction = ray.direction - 2 * (ray.direction.dot(N)) * N;
                reflect_direction.normalize();

                auto kr = fresnel(ray.direction, intersection.normal, _ior_);
                Ray reflection_ray(hit_point, reflect_direction);
                // 递归调用 cast_ray 函数
                hitcolor = cast_ray(reflection_ray, scene, depth + 1) * kr;
                // printf("kr = %f\n", kr);
                break;
            }
            case MaterialType::DIFFUSE_AND_GLOSSY:{
                // 计算漫反射和镜面反射光线
                Vector3f N = intersection.normal.normalized();
                // printf("N: %f %f %f\n", N[0], N[1], N[2]);
                Vector3f _hitcolor = Vector3f(0.0f, 0.0f, 0.0f);
                for (const auto& light : scene.lights) {
                    Vector3f L = (light.position - hit_point).normalized(); // 光线方向
                    
                    auto shadow_result = trace(Ray(hit_point, L), scene); // 计算阴影
                    if (shadow_result.has_value()) continue;    // 如果有阴影则跳过

                    Vector3f diffuse = material.diffuse * std::max(0.0f, N.dot(L));// 漫反射
                    Vector3f V = -ray.direction.normalized(); // 视线方向
                    Vector3f H = (L + V).normalized(); // 半角向量
                    Vector3f specular = material.specular * std::pow(std::max(0.0f, intersection.normal.normalized().dot(H)), material.shininess);
                    float light_attenuation = (light.position - hit_point).norm();
                    light_attenuation = 1 / (light_attenuation * light_attenuation);
                    _hitcolor += light.intensity * (diffuse + specular) * light_attenuation;
                    (void)light_attenuation;
                }
                hitcolor = _hitcolor;                
                break;
            }

        }
    }
    return hitcolor;
}
