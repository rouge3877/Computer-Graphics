float WhittedRenderer::fresnel(const Vector3f& I, const Vector3f& N, const float& ior) 
{
    // 确保入射光线与法线的方向一致
    float cos_theta_i = std::max(-1.0f, std::min(1.0f, I.dot(N)));
    float ni = 1.0f;
    float nt = ior;

    // 如果入射光线与法线的夹角为钝角，则交换折射率
    if (cos_theta_i < 0) {
        cos_theta_i = -cos_theta_i; // 取入射角的绝对值
    } else {
        std::swap(ni, nt);
    }

    float sin_theta_t_square = ni * ni * (1.0f - cos_theta_i * cos_theta_i) / (nt * nt);
    // 如果全反射，则返回1.0
    if (sin_theta_t_square > 1.0f) {
        return 1.0f;
    }

    float cos_theta_t = std::sqrt(1.0f - sin_theta_t_square);

    float R_parl = std::pow((nt * cos_theta_i) - (ni * cos_theta_t)) / ((nt * cos_theta_i) + (ni * cos_theta_t), 2);
    float R_perp = std::pow((ni * cos_theta_i) - (nt * cos_theta_t)) / ((ni * cos_theta_i) + (nt * cos_theta_t), 2);

    return 0.5f * (R_parl + R_perp);
}

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

    return payload.has_value() ? std::make_optional(std::make_tuple(payload.value(), material))
                               : std::nullopt;
}

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

    auto _ior_ = 135.0f; // a high ior means almost mirror reflection
    if(result.has_value()){
        auto [intersection, material] = result.value();
        Vector3f hit_point = ray.origin + ray.direction * intersection.t;
        hit_point += intersection.normal * EPSILON;
        
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