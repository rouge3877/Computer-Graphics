void Scene::simulation_update()
{
    // 这次模拟的总时长不是上一帧的时长，而是上一帧时长与之前帧剩余时长的总和，
    // 即上次调用 simulation_update 到现在过了多久。

    // 以固定的时间步长 (time_step) 循环模拟物体运动，每模拟一步，模拟总时长就减去一个
    // time_step ，当总时长不够一个 time_step 时停止模拟。

    // 根据刚才模拟时间步的数量，更新最后一次调用 simulation_update 的时间 (last_update)。

    auto sim_begin_clock = steady_clock::now();
    auto duration_time = duration(sim_begin_clock - last_update).count();

    while (duration_time > time_step) {
        for (auto object : all_objects) object->update(all_objects);
        duration_time -= time_step;
    }
    
    last_update = sim_begin_clock - 
                duration_cast<decltype(last_update)::duration>(duration(duration_time));
}

KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState next;
    next.acceleration = current.acceleration;
    next.position = current.position + time_step * current.velocity;
    next.velocity = current.velocity + time_step * current.acceleration;
#ifdef DEBUG
    printf("Forward Euler Step(%.2f): \n\tposition = (%.2f, %.2f, %.2f)\n\tvelocity = (%.2f, %.2f, %.2f)\n", time_step,
           next.position.x(), next.position.y(), next.position.z(),
           next.velocity.x(), next.velocity.y(), next.velocity.z());
    printf("Delta Position:\n\t p_x: %.2f = %.2f + %.2f * %.2f\n\t p_y: %.2f = %.2f + %.2f * %.2f\n\t p_z: %.2f = %.2f + %.2f * %.2f\n",
           next.position.x(), current.position.x(), time_step, current.velocity.x(),
           next.position.y(), current.position.y(), time_step, current.velocity.y(),
           next.position.z(), current.position.z(), time_step, current.velocity.z());
    printf("Delta Velocity:\n\t v_x: %.2f = %.2f + %.2f * %.2f\n\t v_y: %.2f = %.2f + %.2f * %.2f\n\t v_z: %.2f = %.2f + %.2f * %.2f\n",
           next.velocity.x(), current.velocity.x(), time_step, current.acceleration.x(),
           next.velocity.y(), current.velocity.y(), time_step, current.acceleration.y(),
           next.velocity.z(), current.velocity.z(), time_step, current.acceleration.z());
#endif
    return next;
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
    for (auto object : all_objects) {
        (void)object;

        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            if (BVH_for_collision) {
            } else {
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
    prev_state = current_state;
    velocity = next_state.velocity;
    force = next_state.acceleration * mass;

}