#include "object.h"

#include <array>
#include <optional>

#ifdef _WIN32
#include <Windows.h>
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fmt/format.h>

#include "../utils/math.hpp"
#include "../utils/ray.h"
#include "../simulation/solver.h"
#include "../utils/logger.h"

using Eigen::Matrix4f;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using std::array;
using std::make_unique;
using std::optional;
using std::string;
using std::vector;

bool Object::BVH_for_collision   = false;
size_t Object::next_available_id = 0;
std::function<KineticState(const KineticState&, const KineticState&)> Object::step =
    forward_euler_step;

Object::Object(const string& object_name)
    : name(object_name), center(0.0f, 0.0f, 0.0f), scaling(1.0f, 1.0f, 1.0f),
      rotation(1.0f, 0.0f, 0.0f, 0.0f), velocity(0.0f, 0.0f, 0.0f), force(0.0f, 0.0f, 0.0f),
      mass(1.0f), BVH_boxes("BVH", GL::Mesh::highlight_wireframe_color)
{
    visible  = true;
    modified = false;
    id       = next_available_id;
    ++next_available_id;
    bvh                      = make_unique<BVH>(mesh);
    const string logger_name = fmt::format("{} (Object ID: {})", name, id);
    logger                   = get_logger(logger_name);
}

Matrix4f Object::model() {
    Matrix4f scaleMatrix = Matrix4f::Identity();
    Matrix4f rotationMatrix = Matrix4f::Identity();
    Matrix4f translationMatrix = Matrix4f::Identity();

    // Create the translation matrix
    translationMatrix.block<3, 1>(0, 3) = center;

    // Create the scaling matrix
    scaleMatrix(0, 0) = scaling.x();
    scaleMatrix(1, 1) = scaling.y();
    scaleMatrix(2, 2) = scaling.z();
 
    // Create the rotation matrix
    const Quaternionf& r = rotation;
    rotationMatrix.block<3, 3>(0, 0) = r.toRotationMatrix();

# ifdef DEBUG
    // Output the matrix by using logger
    logger->info("scaling matrix: \n{}", scaleMatrix);
    logger->info("rotation matrix: \n{}", rotationMatrix);  
    logger->info("center matrix: \n{}", translationMatrix);
# endif

    Matrix4f modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;
    return modelMatrix;
}

void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    (void)next_state;
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    center = next_state.position;
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。

    Matrix4f this_transform = this->model();
    for (auto object : all_objects) {
        if(object == this) continue;

        Matrix4f object_transform = object->model();
        
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            Vector3f this_v0 = (this_transform * mesh.vertex(v_indices[0]).homogeneous()).block<3, 1>(0, 0);
            Vector3f this_v1 = (this_transform * mesh.vertex(v_indices[1]).homogeneous()).block<3, 1>(0, 0);

            Ray this_ray = Ray{this_v0, (this_v1 - this_v0).normalized()};
            std::optional<Intersection> intersection;
            if (BVH_for_collision) {
                intersection = object->bvh->intersect(this_ray, object->mesh, object_transform);
            } else {
                intersection =  naive_intersect(this_ray, object->mesh, object_transform);
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
            if(intersection != std::nullopt && intersection->t <= (this_v1 - this_v0).norm()){
                next_state.position = current_state.position;
                // 在碰撞过程中，为什么冲量 j_r ​是沿着法向 n 而不是沿着物体的运动方向
                
                float impulse = -2.0f * (next_state.velocity - object->velocity).dot(intersection->normal) / (1 / mass + 1 / object->mass);
                next_state.velocity = next_state.velocity + impulse / mass * intersection->normal;
                object->velocity = object->velocity - impulse / object->mass * intersection->normal;
                break;
            }
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    center = next_state.position;
    velocity = next_state.velocity;
    prev_state = current_state;
}

void Object::render(const Shader& shader, WorkingMode mode, bool selected)
{
    if (modified) {
        mesh.VAO.bind();
        mesh.vertices.to_gpu();
        mesh.normals.to_gpu();
        mesh.edges.to_gpu();
        mesh.edges.release();
        mesh.faces.to_gpu();
        mesh.faces.release();
        mesh.VAO.release();
    }
    modified = false;
    // Render faces anyway.
    unsigned int element_flags = GL::Mesh::faces_flag;
    if (mode == WorkingMode::MODEL) {
        // For *Model* mode, only the selected object is rendered at the center in the world.
        // So the model transform is the identity matrix.
        shader.set_uniform("model", I4f);
        shader.set_uniform("normal_transform", I4f);
        element_flags |= GL::Mesh::vertices_flag;
        element_flags |= GL::Mesh::edges_flag;
    } else {
        Matrix4f model = this->model();
        shader.set_uniform("model", model);
        shader.set_uniform("normal_transform", (Matrix4f)(model.inverse().transpose()));
    }
    // Render edges of the selected object for modes with picking enabled.
    if (check_picking_enabled(mode) && selected) {
        element_flags |= GL::Mesh::edges_flag;
    }
    mesh.render(shader, element_flags);
}

void Object::rebuild_BVH()
{
    bvh->recursively_delete(bvh->root);
    bvh->build();
    BVH_boxes.clear();
    refresh_BVH_boxes(bvh->root);
    BVH_boxes.to_gpu();
}

void Object::refresh_BVH_boxes(BVHNode* node)
{
    if (node == nullptr) {
        return;
    }
    BVH_boxes.add_AABB(node->aabb.p_min, node->aabb.p_max);
    refresh_BVH_boxes(node->left);
    refresh_BVH_boxes(node->right);
}
