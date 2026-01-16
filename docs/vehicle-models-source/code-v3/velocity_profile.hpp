// velocity_profile.hpp
#pragma once
#include <vector>
#include <functional>

namespace nhm {

struct VelPoint {
    double s;      // arc length [m]
    double kappa;  // curvature [1/m]
};

struct VelLimits {
    double v_max{10.0};       // [m/s] max speed cap
    double a_t_max{2.0};      // [m/s^2] tangential accel bound
    double a_lat_max{3.0};    // [m/s^2] lateral acceleration bound -> v^2*|kappa| <= a_lat_max
    double j_t_max{4.0};      // [m/s^3] tangential jerk bound (optional)
};

struct TimeProfileSample {
    double s;  // [m]
    double v;  // [m/s]
    double t;  // [s]
};

// Discrete time-optimal parameterization along a path with curvature constraints.
// Implements a standard forward/backward pass for accel bounds, then a jerk-smoothing pass.
// Returns dense (s_i, v_i, t_i).
std::vector<TimeProfileSample>
optimizeVelocityProfile(const std::vector<VelPoint>& path,
                        const VelLimits& lim);

} // namespace nhm
