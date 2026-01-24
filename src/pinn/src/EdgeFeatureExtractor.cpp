#include "pinn/EdgeFeatureExtractor.hpp"
#include <cmath>
#include <iostream>

namespace pinn {

std::vector<float> EdgeFeatureExtractor::extract(
    const PrimitiveInfo& prim,
    float v0,
    float mu,
    const VehicleParams& vehicle
) {
    std::vector<float> features;
    features.reserve(10); // We expect 10 features based on schema

    // 1. Length (arc length) [m]
    features.push_back(prim.length);

    // 2. Delta Yaw [rad]
    // Normalize to [-pi, pi] is usually good practice, but for short primitives raw might be fine.
    // However, let's stick to the raw difference as it captures the turning magnitude.
    float delta_yaw = prim.end_yaw - prim.start_yaw;
    // Handle wrap-around if necessary, though primitives usually don't cross pi boundary ambiguously.
    // Ideally, the planner ensures continuous yaw. 
    // Normalized check:
    while (delta_yaw > M_PI) delta_yaw -= 2.0f * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0f * M_PI;
    features.push_back(delta_yaw);

    // 3 & 4. Delta X, Delta Y in LOCAL frame (aligned with start pose)
    double dx_global = prim.end_x - prim.start_x;
    double dy_global = prim.end_y - prim.start_y;
    double c = std::cos(prim.start_yaw);
    double s = std::sin(prim.start_yaw);

    // Rotation: [ dx_local ] = [  c   s ] [ dx_global ]
    //           [ dy_local ]   [ -s   c ] [ dy_global ]
    float dx_local = static_cast<float>(dx_global * c + dy_global * s);
    float dy_local = static_cast<float>(-dx_global * s + dy_global * c);

    features.push_back(dx_local);
    features.push_back(dy_local);

    // 5. Mean Curvature [1/m]
    // kappa = d_theta / d_s
    float mean_curvature = 0.0f;
    if (prim.length > 1e-4f) {
        mean_curvature = delta_yaw / prim.length;
    }
    features.push_back(mean_curvature);

    // 6. Max Curvature [1/m]
    // Provided by PrimitiveInfo or approximate with mean (for circular arcs mainly)
    features.push_back(prim.max_curvature);

    // 7. Initial Velocity v0 [m/s]
    features.push_back(v0);

    // 8. Friction Coefficient mu
    features.push_back(mu);

    // 9. Wheelbase [m]
    features.push_back(vehicle.wheelbase);

    // 10. Steering Limit [rad]
    features.push_back(vehicle.steering_limit);

    return features;
}

} // namespace pinn
