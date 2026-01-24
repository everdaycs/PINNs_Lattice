#ifndef PINN_EDGE_FEATURE_EXTRACTOR_HPP
#define PINN_EDGE_FEATURE_EXTRACTOR_HPP

#include <vector>
#include <cmath>

namespace pinn {

struct VehicleParams {
    float wheelbase = 0.65f;
    float steering_limit = 0.4189f;
};

// Raw primitive info passed from Lattice Planner
struct PrimitiveInfo {
    float length;
    float start_x, start_y, start_yaw;
    float end_x, end_y, end_yaw;
    float max_curvature;
    // ...
};

class EdgeFeatureExtractor {
public:
    static std::vector<float> extract(
        const PrimitiveInfo& prim,
        float v0,
        float mu,
        const VehicleParams& vehicle
    );
};

} // namespace pinn

#endif
