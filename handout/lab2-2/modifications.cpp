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

bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    auto compute_barycentric_2d_res = compute_barycentric_2d(x, y, vertices);
    float alpha = std::get<0>(compute_barycentric_2d_res);
    float beta = std::get<1>(compute_barycentric_2d_res);
    float gamma = std::get<2>(compute_barycentric_2d_res);

    return alpha >= 0 && beta >= 0 && gamma >= 0;
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

// 光栅化渲染器的渲染调用接口
void RasterizerRenderer::render(const Scene& scene)
{
    Uniforms::width       = static_cast<int>(width);
    Uniforms::height      = static_cast<int>(height);
    Context::frame_buffer = FrameBuffer(Uniforms::width, Uniforms::height);
    // clear Color Buffer & Depth Buffer & rendering_res
    Context::frame_buffer.clear(BufferType::Color | BufferType::Depth);
    this->rendering_res.clear();
    // run time statistics
    time_point begin_time                  = steady_clock::now();
    Camera cam                             = scene.camera;
    vertex_processor.vertex_shader_ptr     = vertex_shader;
    fragment_processor.fragment_shader_ptr = phong_fragment_shader;
    for (const auto& group : scene.groups) {
        for (const auto& object : group->objects) {
            Context::vertex_finish     = false;
            Context::rasterizer_finish = false;
            Context::fragment_finish   = false;

            std::vector<std::thread> workers;
            for (int i = 0; i < n_vertex_threads; ++i) {
                workers.emplace_back(&VertexProcessor::worker_thread, &vertex_processor);
            }
            for (int i = 0; i < n_rasterizer_threads; ++i) {
                workers.emplace_back(&Rasterizer::worker_thread, &rasterizer);
            }
            for (int i = 0; i < n_fragment_threads; ++i) {
                workers.emplace_back(&FragmentProcessor::worker_thread, &fragment_processor);
            }

            // set Uniforms for vertex shader
            Uniforms::MVP         = cam.projection() * cam.view() * object->model();
            Uniforms::inv_trans_M = object->model().inverse().transpose();
            Uniforms::width       = static_cast<int>(this->width);
            Uniforms::height      = static_cast<int>(this->height);
            // To do: 同步
            // {
            //     std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            //     Context::vertex_shader_output_queue = std::queue<VertexShaderPayload>();
            // }
            // {
            //     std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
            //     Context::rasterizer_output_queue = std::queue<FragmentShaderPayload>();
            // }
            Uniforms::material = object->mesh.material;
            Uniforms::lights   = scene.lights;
            Uniforms::camera   = scene.camera;

            // input object->mesh's vertices & faces & normals data
            const std::vector<float>& vertices     = object->mesh.vertices.data;
            const std::vector<unsigned int>& faces = object->mesh.faces.data;
            const std::vector<float>& normals      = object->mesh.normals.data;
            size_t num_faces                       = faces.size();

            // process vertices
            for (size_t i = 0; i < num_faces; i += 3) {
                for (size_t j = 0; j < 3; j++) {
                    size_t idx = faces[i + j];
                    vertex_processor.input_vertices(
                        Vector4f(vertices[3 * idx], vertices[3 * idx + 1], vertices[3 * idx + 2],
                                 1.0f),
                        Vector3f(normals[3 * idx], normals[3 * idx + 1], normals[3 * idx + 2]));
                }
            }
            vertex_processor.input_vertices(Eigen::Vector4f(0, 0, 0, -1.0f),
                                            Eigen::Vector3f::Zero());
            for (auto& worker : workers) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
        }
    }

    time_point end_time         = steady_clock::now();
    duration rendering_duration = end_time - begin_time;

    this->logger->info("rendering (single thread) takes {:.6f} seconds",
                       rendering_duration.count());

    for (long unsigned int i = 0; i < Context::frame_buffer.depth_buffer.size(); i++) {
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].x()));
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].y()));
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].z()));
    }
}

void VertexProcessor::worker_thread()
{
    while (true) {
        VertexShaderPayload payload;
        {
            if (vertex_queue.empty()) {
                printf("\n"); // Avoid the loop optimization bug
                continue;
            }
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (vertex_queue.empty()) {
                continue;
            }
            payload = vertex_queue.front();
            vertex_queue.pop();
        }
        if (payload.world_position.w() == -1.0f) {
            Context::vertex_finish = true;
            return;
        }
        VertexShaderPayload output_payload = vertex_shader_ptr(payload);
        {
            std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            Context::vertex_shader_output_queue.push(output_payload);
        }
    }
}

void FragmentProcessor::worker_thread()
{
    while (true) {
        FragmentShaderPayload fragment;
        {
            if (Context::rasterizer_finish && Context::rasterizer_output_queue.empty()) {
                Context::fragment_finish = true;
                return;
            }
            if (Context::rasterizer_output_queue.empty()) {
                printf("\n"); // Avoid the loop optimization bug
                continue;
            }
            std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
            if (Context::rasterizer_output_queue.empty()) {
                continue;
            }
            fragment = Context::rasterizer_output_queue.front();
            Context::rasterizer_output_queue.pop();
        }
        int index = (Uniforms::height - 1 - fragment.y) * Uniforms::width + fragment.x;
        if (fragment.depth > Context::frame_buffer.depth_buffer[index]) {
            continue;
        }
        fragment.color =
            fragment_shader_ptr(fragment, Uniforms::material, Uniforms::lights, Uniforms::camera);
        Context::frame_buffer.set_pixel(index, fragment.depth, fragment.color);
    }
}
