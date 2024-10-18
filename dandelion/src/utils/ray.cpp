#include "ray.h"

#include <cmath>
#include <array>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "../utils/math.hpp"

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::numeric_limits;
using std::optional;
using std::size_t;

constexpr float infinity = 1e5f;
constexpr float eps      = 1e-5f;

Intersection::Intersection() : t(numeric_limits<float>::infinity()), face_index(0)
{
}

Ray generate_ray(int width, int height, int x, int y, Camera& camera, float depth)
{
    float fov_y = radians(camera.fov_y_degrees);
    float image_plane_height = 2.0f * tan(fov_y / 2.0f);
    float aspect_ratio = static_cast<float>(width) / height;
    float image_plane_width = image_plane_height * aspect_ratio;

    float pixel_ndc_x = (static_cast<float>(x) + 0.5f) / width;
    float pixel_ndc_y = (static_cast<float>(y) + 0.5f) / height;

    float pixel_screen_x = 2.0f * pixel_ndc_x - 1.0f;
    float pixel_screen_y = 1.0f - 2.0f * pixel_ndc_y;

    float pixel_camera_x = pixel_screen_x * image_plane_width / 2.0f;
    float pixel_camera_y = pixel_screen_y * image_plane_height / 2.0f;

    Vector4f pixel_camera_space(pixel_camera_x, pixel_camera_y, -depth, 1.0f);
    Matrix4f inv_view = camera.view().inverse();
    Vector4f pixel_world_space = inv_view * pixel_camera_space;

    Vector3f ray_origin = camera.position;
    Vector3f ray_direction = (pixel_world_space.head<3>() / pixel_world_space.w() - ray_origin).normalized();

    return {ray_origin, ray_direction};
}

optional<Intersection> ray_triangle_intersect(const Ray& ray, const GL::Mesh& mesh, size_t index)
{
    // these lines below are just for compiling and can be deleted
    (void)ray;
    (void)mesh;
    (void)index;
    // these lines above are just for compiling and can be deleted
    Intersection result;
    
    if (result.t - infinity < -eps) {
        return result;
    } else {
        return std::nullopt;
    }
}

optional<Intersection> naive_intersect(const Ray& ray, const GL::Mesh& mesh, const Matrix4f model)
{
    optional<Intersection> result;
    float min_t_in_scoop = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < mesh.faces.count(); i++) {
        const auto& face = mesh.face(i);
        float this_t;

        Vector3f this_vertex_a = (model * mesh.vertex(face[0]).homogeneous()).hnormalized();
        Vector3f this_vertex_b = (model * mesh.vertex(face[1]).homogeneous()).hnormalized();
        Vector3f this_vertex_c = (model * mesh.vertex(face[2]).homogeneous()).hnormalized();
        Vector3f this_normal = (this_vertex_b - this_vertex_a).cross(this_vertex_c - this_vertex_a).normalized();

        if (std::abs(this_normal.dot(ray.direction)) < eps) {
            continue; // avoid devide ZERO
        }else{
            this_t = (this_vertex_a - ray.origin).dot(this_normal) / this_normal.dot(ray.direction);
            if (this_t < eps || this_t >= min_t_in_scoop) continue;
        }

        Vector3f cross_point = ray.origin + this_t * ray.direction;

        if (this_normal.dot((this_vertex_b - this_vertex_a).cross(cross_point - this_vertex_a)) < 0) continue;
        else if (this_normal.dot((this_vertex_c - this_vertex_b).cross(cross_point - this_vertex_b)) < 0) continue;
        else if (this_normal.dot((this_vertex_a - this_vertex_c).cross(cross_point - this_vertex_c)) < 0) continue;
        else if (this_t < min_t_in_scoop) {
            Intersection _result;
            min_t_in_scoop = this_t;
            _result.t = this_t;
            _result.barycentric_coord = cross_point;
            _result.normal = this_normal;
            result = _result;
        }
    }

    return (min_t_in_scoop < std::numeric_limits<float>::infinity()) ? result : std::nullopt;
}