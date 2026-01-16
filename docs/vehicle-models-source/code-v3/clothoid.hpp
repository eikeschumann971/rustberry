// clothoid.hpp
#pragma once
#include <vector>

namespace nhm {

struct SE2 {
    double x, y, yaw; // [m], [m], [rad]
};

struct ClothoidSeg { // linear curvature segment
    double x0, y0, yaw0; // start pose
    double k0;           // initial curvature
    double sigma;        // curvature rate
    double L;            // length
};

// Fit G2 clothoid segments through a sequence of SE2 samples (coarse path).
// Returns a chain of clothoids approximating the path with curvature continuity.
std::vector<ClothoidSeg> fitClothoidChain(const std::vector<SE2>& waypoints,
                                          double k_max,
                                          double sigma_max);

// Sample clothoid chain at spacing ds -> dense SE2 + curvature
struct ClothoidSample { double s, x, y, yaw, kappa; };
std::vector<ClothoidSample> sampleClothoids(const std::vector<ClothoidSeg>& segs,
                                            double ds);

// Conservative trailer-aware bounds for tractor curvature and curvature rate.
// Returns {k_max, sigma_max}. Heuristic based on keeping |beta| <= beta_max and lateral accel <= a_lat_max.
std::pair<double,double> trailerAwareKappaSigmaBounds(double L0, double L1, double a_off,
                                                     double beta_max,
                                                     double v_ref,
                                                     double a_lat_max);


} // namespace nhm
