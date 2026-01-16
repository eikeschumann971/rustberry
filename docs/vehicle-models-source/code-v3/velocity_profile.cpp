// velocity_profile.cpp
#include "velocity_profile.hpp"
#include <algorithm>
#include <cmath>

namespace nhm {

static inline double v_from_lat(double a_lat_max, double kappa){
    double ak = std::abs(kappa);
    if (ak < 1e-9) return 1e9; // practically unbounded by curvature
    return std::sqrt(a_lat_max / ak);
}

std::vector<TimeProfileSample>
optimizeVelocityProfile(const std::vector<VelPoint>& pts,
                        const VelLimits& lim) {
    if (pts.size() < 2) return {};

    const size_t N = pts.size();
    std::vector<double> s(N), k(N), vmax(N), v(N);
    s[0]=pts[0].s; k[0]=pts[0].kappa;
    for (size_t i=1;i<N;++i){ s[i]=pts[i].s; k[i]=pts[i].kappa; }

    // Per-point max speed from curvature + hard cap
    for (size_t i=0;i<N;++i){
        vmax[i] = std::min(lim.v_max, v_from_lat(lim.a_lat_max, k[i]));
        if (!std::isfinite(vmax[i])) vmax[i] = lim.v_max;
    }

    // Initialize with curvature-limited speed
    v = vmax;

    // Forward pass: acceleration bound v_{i+1}^2 <= v_i^2 + 2 a_max ds
    for (size_t i=0;i+1<N;++i){
        double ds = std::max(1e-6, s[i+1]-s[i]);
        double vnext2 = v[i]*v[i] + 2.0*lim.a_t_max*ds;
        double vnext = std::sqrt(std::max(0.0, vnext2));
        v[i+1] = std::min(v[i+1], vnext);
    }

    // Backward pass
    for (size_t i=N-1;i>0;--i){
        double ds = std::max(1e-6, s[i]-s[i-1]);
        double vprev2 = v[i]*v[i] + 2.0*lim.a_t_max*ds; // symmetric
        double vprev = std::sqrt(std::max(0.0, vprev2));
        v[i-1] = std::min(v[i-1], vprev);
    }

    // Optional jerk pass: enforce |a_{i+1} - a_i| <= j_max * dt
    if (lim.j_t_max > 0){
        // Iterate a few times for consistency
        for (int it=0; it<3; ++it){
            // compute time stamps
            std::vector<double> t(N,0.0);
            for (size_t i=1;i<N;++i){
                double ds = std::max(1e-6, s[i]-s[i-1]);
                double vavg = std::max(1e-3, 0.5*(v[i]+v[i-1]));
                t[i] = t[i-1] + ds / vavg;
            }
            // accelerations a_t = v dv/ds * v = dv/dt (discrete)
            std::vector<double> a(N,0.0);
            for (size_t i=1;i<N;++i){
                double dt = std::max(1e-6, t[i]-t[i-1]);
                a[i] = (v[i]-v[i-1]) / dt;
            }
            for (size_t i=1;i+1<N;++i){
                double dt = std::max(1e-6, t[i]-t[i-1]);
                double dt2 = std::max(1e-6, t[i+1]-t[i]);
                double amax_delta_prev = lim.j_t_max * dt;
                double amax_delta_next = lim.j_t_max * dt2;
                // limit jump between a[i-1] and a[i]
                double da_prev = a[i]-a[i-1];
                if (std::abs(da_prev) > amax_delta_prev){
                    a[i] = a[i-1] + std::copysign(amax_delta_prev, da_prev);
                }
                double da_next = a[i+1]-a[i];
                if (std::abs(da_next) > amax_delta_next){
                    a[i+1] = a[i] + std::copysign(amax_delta_next, da_next);
                }
            }
            // integrate accelerations back to v (least-squares style smoothing)
            v[0] = std::min(v[0], vmax[0]);
            for (size_t i=1;i<N;++i){
                double dt = std::max(1e-6, t[i]-t[i-1]);
                v[i] = std::clamp(v[i-1] + a[i]*dt, 0.0, vmax[i]);
            }
        }
    }

    // produce output with times
    std::vector<TimeProfileSample> out; out.reserve(N);
    double t=0.0; out.push_back({s[0], v[0], t});
    for (size_t i=1;i<N;++i){
        double ds = std::max(1e-6, s[i]-s[i-1]);
        double vavg = std::max(1e-3, 0.5*(v[i]+v[i-1]));
        t += ds / vavg;
        out.push_back({s[i], v[i], t});
    }
    return out;
}

} // namespace nhm
