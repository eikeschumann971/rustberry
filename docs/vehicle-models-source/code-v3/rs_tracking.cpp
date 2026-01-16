// rs_tracking.cpp
#include "rs_tracking.hpp"
#include <cmath>

namespace nhm {

static inline double norm(double a){ while(a>M_PI) a-=2*M_PI; while(a<-M_PI) a+=2*M_PI; return a; }

std::vector<RSTrackSample> makeReverseAwareReference(const std::vector<double>& xs,
                                                     const std::vector<double>& ys,
                                                     const std::vector<double>& yaws,
                                                     double turningRadius,
                                                     double v_fwd, double v_rev){
    std::vector<RSTrackSample> ref; if (xs.size()<2) return ref;
    double t=0.0; double v=v_fwd;
    for (size_t i=1;i<xs.size();++i){
        double dx=xs[i]-xs[i-1], dy=ys[i]-ys[i-1];
        double ds=std::hypot(dx,dy); if (ds<1e-6) continue;
        double dyaw = norm(yaws[i]-yaws[i-1]);
        double kappa = dyaw / ds; // signed
        // heuristic: if there is a cusp (large |dyaw| with tiny ds), switch gear
        bool cusp = (std::abs(dyaw) > 1.0 && ds < turningRadius*0.2);
        if (cusp) v = (v>0)? -std::abs(v_rev) : std::abs(v_fwd);
        double dt = ds / std::max(1e-3, std::abs(v));
        t += dt;
        ref.push_back({t, xs[i], ys[i], yaws[i], kappa, v});
    }
    return ref;
}

} // namespace nhm
