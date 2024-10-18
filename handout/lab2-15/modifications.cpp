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

void Object::update(vector<Object*>& all_objects)
{
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    this->center = next_state.position;

    // Check collision with other objects.
    for (auto object : all_objects) {
        if(object == this) continue;
        
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            Vector3f this_v0 = (this->model() * mesh.vertex(v_indices[0]).homogeneous()).hnormalized();
            Vector3f this_v1 = (this->model() * mesh.vertex(v_indices[1]).homogeneous()).hnormalized();

            Ray this_edge_ray = Ray{this_v0, (this_v1 - this_v0).normalized()};
            std::optional<Intersection> intersection;
            if (BVH_for_collision) intersection = object->bvh->intersect(this_edge_ray, object->mesh, object->model());
            else intersection = naive_intersect(this_edge_ray, object->mesh, object->model());
            
            if(intersection != std::nullopt && intersection->t <= (this_v1 - this_v0).norm()){
                next_state.position = current_state.position;
                // 在碰撞过程中，为什么冲量 j_r ​是沿着法向 n 而不是沿着物体的运动方向
                // 存在问题：当物体的速度，距离，以及刷新帧率满足一定条件时，交面的法向量会垂直于速度方向
                // 从而导致冲量的方向与速度方向相反，这样会导致物体的速度变为0，从而无法继续运动
                // 初步的分析是，这时候由于物体恰好接壤，因此此时作任意边到另一物体面的交点实际上都是不存在的
                // 已解决：在鉴定存在碰撞后，next_state = step(current_state, next_state);
                // 解决原因：TODO

                float impulse = -2.0f * (next_state.velocity - object->velocity).dot(intersection->normal) / (1 / mass + 1 / object->mass);
                next_state.velocity = next_state.velocity + (impulse / this->mass) * intersection->normal;
                object->velocity = object->velocity - (impulse / object->mass) * intersection->normal;

                next_state = step(current_state, next_state); // Amazing... Who can interpret it!!!
                break;
            }
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    this->center = next_state.position;
    this->velocity = next_state.velocity;
    this->force = next_state.acceleration * mass;
    this->prev_state = current_state;
}