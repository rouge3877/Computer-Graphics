KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    KineticState next = current;
    Vector3f k1_v = current.acceleration;
    // All k2, k3, k4 are the same as k1 in this case
    // Vector3f k2_v = current.acceleration;
    // Vector3f k3_v = current.acceleration;
    // Vector3f k4_v = current.acceleration; 
    next.velocity = current.velocity + (time_step / 6) * (k1_v + 2 * k1_v + 2 * k1_v + k1_v);
   
    Vector3f k1_x = current.velocity;
    Vector3f k2_x = current.velocity + (time_step / 2) * current.acceleration;
    Vector3f k3_x = current.velocity + (time_step / 2) * current.acceleration;
    Vector3f k4_x = current.velocity + time_step * current.acceleration;
    next.position = current.position + (time_step / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);

#ifdef DEBUG
    printf("Runge-Kutta Step(%.2f): \n\tposition = (%.2f, %.2f, %.2f)\n\tvelocity = (%.2f, %.2f, %.2f)\n", time_step,
            next.position.x(), next.position.y(), next.position.z(),
            next.velocity.x(), next.velocity.y(), next.velocity.z());
            
    printf("Delta Position:\n\t p_x: %2.f = %.2f + (%.2f / 6) * (%.2f + 2 * %.2f + 2 * %.2f + %.2f)\n",
            next.position.x(), current.position.x(), time_step, k1_x.x(), k2_x.x(), k3_x.x(), k4_x.x());
    printf("\t p_y: %.2f = %.2f + (%.2f / 2) * %.2f + (%.2f / 2) * %.2f + %.2f\n", 
            next.position.y(), current.position.y(), time_step, k1_x.y(), k2_x.y(), k3_x.y(), k4_x.y());
    printf("\t p_z: %.2f = %.2f + (%.2f / 2) * %.2f + (%.2f / 2) * %.2f + %.2f\n", 
            next.position.z(), current.position.z(), time_step, k1_x.z(), k2_x.z(), k3_x.z(), k4_x.z());

    printf("Delta Velocity:\n\t v_x: %.2f = %.2f + (%.2f / 6) * (%.2f + 2 * %.2f + 2 * %.2f + %.2f)\n",
            next.velocity.x(), current.velocity.x(), time_step, k1_v.x(), k1_v.x(), k1_v.x(), k1_v.x());
    printf("\t v_y: %.2f = %.2f + (%.2f / 6) * (%.2f + 2 * %.2f + 2 * %.2f + %.2f)\n",
            next.velocity.y(), current.velocity.y(), time_step, k1_v.y(), k1_v.y(), k1_v.y(), k1_v.y());
    printf("\t v_z: %.2f = %.2f + (%.2f / 6) * (%.2f + 2 * %.2f + 2 * %.2f + %.2f)\n",
            next.velocity.z(), current.velocity.z(), time_step, k1_v.z(), k1_v.z(), k1_v.z(), k1_v.z());
#endif
    return next;
}

KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    KineticState next;
    next.acceleration = current.acceleration;
    next.velocity = current.velocity + time_step * next.acceleration;
    next.position = current.position + time_step * next.velocity;
#ifdef DEBUG
    printf("Backward Euler Step(%.2f): \n\tposition = (%.2f, %.2f, %.2f)\n\tvelocity = (%.2f, %.2f, %.2f)\n", time_step,
            next.position.x(), next.position.y(), next.position.z(),
            next.velocity.x(), next.velocity.y(), next.velocity.z());
    printf("Delta Position:\n\t p_x: %.2f = %.2f + %.2f * %.2f\n\t p_y: %.2f = %.2f + %.2f * %.2f\n\t p_z: %.2f = %.2f + %.2f * %.2f\n",
            next.position.x(), current.position.x(), time_step, next.velocity.x(),
            next.position.y(), current.position.y(), time_step, next.velocity.y(),
            next.position.z(), current.position.z(), time_step, next.velocity.z());
    printf("Delta Velocity:\n\t v_x: %.2f = %.2f + %.2f * %.2f\n\t v_y: %.2f = %.2f + %.2f * %.2f\n\t v_z: %.2f = %.2f + %.2f * %.2f\n",
            next.velocity.x(), current.velocity.x(), time_step, current.acceleration.x(),
            next.velocity.y(), current.velocity.y(), time_step, current.acceleration.y(),
            next.velocity.z(), current.velocity.z(), time_step, current.acceleration.z());
#endif
    return next;
}

KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{
    (void)previous;
    KineticState next;
    next.acceleration = current.acceleration;
    next.velocity = current.velocity + time_step * current.acceleration;
    next.position = current.position + time_step * next.velocity;
#ifdef DEBUG
    printf("Symplectic Euler Step(%.2f): \n\tposition = (%.2f, %.2f, %.2f)\n\tvelocity = (%.2f, %.2f, %.2f)\n", time_step,
            next.position.x(), next.position.y(), next.position.z(),
            next.velocity.x(), next.velocity.y(), next.velocity.z());
    printf("Delta Position:\n\t p_x: %.2f = %.2f + %.2f * %.2f\n\t p_y: %.2f = %.2f + %.2f * %.2f\n\t p_z: %.2f = %.2f + %.2f * %.2f\n",
            next.position.x(), current.position.x(), time_step, next.velocity.x(),
            next.position.y(), current.position.y(), time_step, next.velocity.y(),
            next.position.z(), current.position.z(), time_step, next.velocity.z());
    printf("Delta Velocity:\n\t v_x: %.2f = %.2f + %.2f * %.2f\n\t v_y: %.2f = %.2f + %.2f * %.2f\n\t v_z: %.2f = %.2f + %.2f * %.2f\n",
            next.velocity.x(), current.velocity.x(), time_step, current.acceleration.x(),
            next.velocity.y(), current.velocity.y(), time_step, current.acceleration.y(),
            next.velocity.z(), current.velocity.z(), time_step, current.acceleration.z());
#endif
    return next;
}
