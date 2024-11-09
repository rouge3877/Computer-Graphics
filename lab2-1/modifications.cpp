void Toolbar::layout_mode(Scene& scene)
{
    if (ImGui::BeginTabItem("Layout")) {
        if (mode != WorkingMode::LAYOUT) {
            on_selection_canceled();
            mode = WorkingMode::LAYOUT;
        }
        scene_hierarchies(scene);

        Object* selected_object = scene.selected_object;
        if (selected_object != nullptr) {
            material_editor(selected_object->mesh.material);
            ImGui::SeparatorText("Transform");
            ImGui::Text("Translation");
            ImGui::PushID("Translation##");
            Vector3f& center = selected_object->center;
            xyz_drag(&center.x(), &center.y(), &center.z(), POSITION_UNIT);
            ImGui::PopID();

            ImGui::Text("Scaling");
            ImGui::PushID("Scaling##");
            Vector3f& scaling = selected_object->scaling;
            xyz_drag(&scaling.x(), &scaling.y(), &scaling.z(), SCALING_UNIT);
            ImGui::PopID();

            const Quaternionf& r             = selected_object->rotation;
            auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());
            ImGui::Text("Rotation (ZYX Euler)");
            ImGui::PushID("Rotation##");
            ImGui::PushItemWidth(0.3f * ImGui::CalcItemWidth());
            ImGui::DragFloat("pitch", &x_angle, ANGLE_UNIT, -180.0f, 180.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::DragFloat("yaw", &y_angle, ANGLE_UNIT, -90.0f, 90.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::SameLine();
            ImGui::DragFloat("roll", &z_angle, ANGLE_UNIT, -180.0f, 180.0f, "%.1f deg",
                             ImGuiSliderFlags_AlwaysClamp);
            ImGui::PopItemWidth();
            ImGui::PopID();

            // Following given Expression is used to transform euler angles to quaternion by using AngleAxisf(Eigen)
            //
            // selected_object->rotation = AngleAxisf(radians(x_angle), Vector3f::UnitX()) *
            //                             AngleAxisf(radians(y_angle), Vector3f::UnitY()) *
            //                             AngleAxisf(radians(z_angle), Vector3f::UnitZ());
            //
            // TODO: Implement the transformation from euler angles to quaternion without AngleAxisf
            // 
            // Solution:
            //      A quaternion has two part, a scalar part(real-`w`) and a vector part(imaginary-`[i,j,k]`),
            //      We can associate a quaternion with a rotation around an axis by the following expression[1]
            //          q_x = \sin(\theta_x/2) * cos(\beta_x)
            //          q_y = \sin(\theta_y/2) * cos(\beta_y)
            //          q_z = \sin(\theta_z/2) * cos(\beta_z)
            //      In our case, x => pitch, y => yaw, z => roll
            //          q_x = [\cos(\pitch/2), \sin(\pitch/2), 0, 0]
            //          q_y = [\cos(\yaw/2), 0, \sin(\yaw/2), 0]
            //          q_z = [\cos(\roll/2), 0, 0, \sin(\roll/2)]
            //      Thus, the quaternion can be represented as the following quaternion multiplication:
            //          q = q_x * q_y * q_z
            //
            // Citation: 
            //      - [1] https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            //      - [2] https://ntrs.nasa.gov/api/citations/19770019231/downloads/19770019231.pdf
            //

            auto x_angle_rad = radians(x_angle);
            auto y_angle_rad = radians(y_angle);
            auto z_angle_rad = radians(z_angle);

            Quaternionf q_x(std::cos(x_angle_rad / 2), std::sin(x_angle_rad / 2), 0, 0);
            Quaternionf q_y(std::cos(y_angle_rad / 2), 0, std::sin(y_angle_rad / 2), 0);
            Quaternionf q_z(std::cos(z_angle_rad / 2), 0, 0, std::sin(z_angle_rad / 2));

            selected_object->rotation = q_x * q_y * q_z;
        }
        ImGui::EndTabItem();
    }
}