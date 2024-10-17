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
    Intersection result;
    result.t = infinity + 1;
    for (size_t i = 0; i < mesh.faces.count(); ++i) {
        // Vertex a, b and c are assumed to be in counterclockwise order.
        // Construct matrix A = [d, a - b, a - c] and solve Ax = (a - origin)
        // Matrix A is not invertible, indicating the ray is parallel with the triangle.
        // Test if alpha, beta and gamma are all between 0 and 1.
        
        std::array<size_t, 3> facesIndex = mesh.face(i);

        Vector3f v0 = (model * mesh.vertex(facesIndex[0]).homogeneous()).head<3>();
        Vector3f v1 = (model * mesh.vertex(facesIndex[1]).homogeneous()).head<3>();
        Vector3f v2 = (model * mesh.vertex(facesIndex[2]).homogeneous()).head<3>();

        Matrix3f M;
        M << -ray.direction, v1 - v0, v2 - v0;

        float det = M.determinant();
        if(std::fabs(det) < eps){
            continue;
        }

        Vector3f res = M.inverse() * (ray.origin - v0);
        float t = res[0], beta = res[1], gamma = res[2], alpha = 1 - gamma - beta;
        if(t < eps || alpha < 0 || beta < 0 || gamma < 0){
            continue;
        }else if(t < result.t){
            result.t = t;
            result.face_index = i;
            result.barycentric_coord = {alpha, beta, gamma};
            result.normal = ((v1 - v0).cross(v2 - v0)).normalized();
        }
    }
    // Ensure result.t is strictly less than the constant `infinity`.
    if (result.t - infinity < -eps) {
        return result;
    }
    return std::nullopt;
}