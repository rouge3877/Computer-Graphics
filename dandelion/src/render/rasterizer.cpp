#include <array>
#include <limits>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "rasterizer.h"
#include "triangle.h"
#include "../utils/math.hpp"

using Eigen::Matrix4f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::fill;
using std::tuple;

void Rasterizer::worker_thread()
{
    while (true) {
        VertexShaderPayload payload;
        Triangle triangle;
        {
            // printf("vertex_finish = %d\n vertex_shader_output_queue.size = %ld\n",
            // Context::vertex_finish, Context::vertex_shader_output_queue.size());
            if (Context::vertex_finish && Context::vertex_shader_output_queue.empty()) {
                Context::rasterizer_finish = true;
                return;
            }
            if (Context::vertex_shader_output_queue.size() < 3) {
                printf("/n"); // Avoid the bug of the loop optimization
                continue;
            }
            std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            for (size_t vertex_count = 0; vertex_count < 3; vertex_count++) {
                payload = Context::vertex_shader_output_queue.front();
                Context::vertex_shader_output_queue.pop();
                if (vertex_count == 0) {
                    triangle.world_pos[0]    = payload.world_position;
                    triangle.viewport_pos[0] = payload.viewport_position;
                    triangle.normal[0]       = payload.normal;
                } else if (vertex_count == 1) {
                    triangle.world_pos[1]    = payload.world_position;
                    triangle.viewport_pos[1] = payload.viewport_position;
                    triangle.normal[1]       = payload.normal;
                } else {
                    triangle.world_pos[2]    = payload.world_position;
                    triangle.viewport_pos[2] = payload.viewport_position;
                    triangle.normal[2]       = payload.normal;
                }
            }
        }
        rasterize_triangle(triangle);
    }
}

float sign(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3)
{
    return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    auto compute_barycentric_2d_res = compute_barycentric_2d(x, y, vertices);
    float alpha = std::get<0>(compute_barycentric_2d_res);
    float beta = std::get<1>(compute_barycentric_2d_res);
    float gamma = std::get<2>(compute_barycentric_2d_res);

    return alpha >= 0 && beta >= 0 && gamma > 0;
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
    float c1 = 0.f, c2 = 0.f, c3 = 0.f;

    Eigen::Vector2f _point = {x, y};
    Eigen::Vector2f _tri_point[3];
    for (int i = 0; i < 3; i++) 
        _tri_point[i] = {v[i].x(), v[i].y()};  
  
    Eigen::Matrix3f _ABC, _ABP, _PBC, _APC;
    _ABC << 1, _tri_point[0].x(), _tri_point[0].y(),
            1, _tri_point[1].x(), _tri_point[1].y(),
            1, _tri_point[2].x(), _tri_point[2].y();
    _ABP << 1, _tri_point[0].x(), _tri_point[0].y(),
            1, _tri_point[1].x(), _tri_point[1].y(),
            1, _point.x(), _point.y();
    _PBC << 1, _point.x(), _point.y(),
            1, _tri_point[1].x(), _tri_point[1].y(),
            1, _tri_point[2].x(), _tri_point[2].y();
    _APC << 1, _tri_point[0].x(), _tri_point[0].y(),
            1, _point.x(), _point.y(),
            1, _tri_point[2].x(), _tri_point[2].y();

    float _ABC_det = _ABC.determinant();
    c1 = _PBC.determinant() / _ABC_det;
    c2 = _APC.determinant() / _ABC_det;
    c3 = _ABP.determinant() / _ABC_det;
    return {c1, c2, c3};
    // Cite: https://en.wikipedia.org/wiki/Barycentric_coordinate_system
}

// 对顶点的某一属性插值
Vector3f Rasterizer::interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1,
                                 const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3,
                                 const Eigen::Vector3f& weight, const float& Z)
{
    Vector3f interpolated_res;
    for (int i = 0; i < 3; i++) {
        interpolated_res[i] = alpha * vert1[i] / weight[0] + beta * vert2[i] / weight[1] +
                              gamma * vert3[i] / weight[2];
    }
    interpolated_res *= Z;
    return interpolated_res;
}

// 对当前三角形进行光栅化
void Rasterizer::rasterize_triangle(Triangle& t)
{
    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. push primitive into fragment queue

    int min_x = (int)ceil(std::max(0.0f, std::min(std::min(t.viewport_pos[0].x(), t.viewport_pos[1].x()), t.viewport_pos[2].x())));
    int max_x = (int)floor(std::min(Context::frame_buffer.width - 1.0f, std::max(std::max(t.viewport_pos[0].x(), t.viewport_pos[1].x()), t.viewport_pos[2].x())));
    int min_y = (int)ceil(std::max(0.0f, std::min(std::min(t.viewport_pos[0].y(), t.viewport_pos[1].y()), t.viewport_pos[2].y())));
    int max_y = (int)floor(std::min(Context::frame_buffer.height - 1.0f, std::max(std::max(t.viewport_pos[0].y(), t.viewport_pos[1].y()), t.viewport_pos[2].y())));

    // FragmentShaderPayload payload;
    Vector3f weight = {t.viewport_pos[0].w(), t.viewport_pos[1].w(), t.viewport_pos[2].w()};
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            if (inside_triangle(x, y, t.viewport_pos)) {
                FragmentShaderPayload payload = {Vector3f::Zero(), Vector3f::Zero(), x, y, 0, Vector3f::Zero()};
                // interpolate depth
                auto [alpha, beta, gamma] = compute_barycentric_2d(x, y, t.viewport_pos);
                float Z = 1 / (alpha / t.viewport_pos[0].w() + beta / t.viewport_pos[1].w() + gamma / t.viewport_pos[2].w());
                Z = (alpha * t.viewport_pos[0].z() / t.viewport_pos[0].w() + beta * t.viewport_pos[1].z() / t.viewport_pos[1].w() + gamma * t.viewport_pos[2].z() / t.viewport_pos[2].w()) * Z;
                int index = (Context::frame_buffer.height - 1 - y) * Context::frame_buffer.width + x;
                if (Z <= Context::frame_buffer.depth_buffer[index]){
                    Context::frame_buffer.depth_buffer[index] = Z;
                    payload.depth = Z;
                    // interpolate vertex position & normal
                    Vector3f interpolated_pos = interpolate(alpha, beta, gamma, t.world_pos[0].head(3), t.world_pos[1].head(3), t.world_pos[2].head(3), weight, Z);
                    Vector3f interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], weight, Z);
                    payload.world_pos = interpolated_pos;
                    payload.world_normal = interpolated_normal;
                }else{
                    continue;
                }
                // push primitive into fragment queue
                std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
                Context::rasterizer_output_queue.push(payload);   
            }
        }
    }
}
