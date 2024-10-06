#include "solver.h"

#include <Eigen/Core>

using Eigen::Vector3f;

// External Force does not changed.

// Function to calculate the derivative of KineticState
KineticState derivative(const KineticState& state)
{
    return KineticState(state.velocity, state.acceleration, Eigen::Vector3f(0, 0, 0));
}

// Function to perform a single Forward Euler step
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

// Function to perform a single Runge-Kutta step
KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    return current;
}

// Function to perform a single Backward Euler step
KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    return current;
}

// Function to perform a single Symplectic Euler step
KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{
    (void)previous;
    return current;
}
