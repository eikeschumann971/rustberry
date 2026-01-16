// rs_tracking.hpp
#pragma once
#include <vector>

namespace nhm {

struct RSTrackSample { double t, x, y, yaw, kappa, v; };

// Given a Reeds–Shepp geometric path (SE2 states), generate forward/reverse aware reference with signed velocity.
// sign is inferred from yaw change sign between samples and segment lengths; simple heuristic: if heading flips with cusp, assign negative v on that segment when backing.
std::vector<RSTrackSample> makeReverseAwareReference(const std::vector<double>& xs,
                                                     const std::vector<double>& ys,
                                                     const std::vector<double>& yaws,
                                                     double turningRadius,
                                                     double v_fwd, double v_rev);

} // namespace nhm
