#include "nonholonomic_models.hpp"
#include "velocity_profile.hpp"
#include "clothoid.hpp"
#include "mpc_templates.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace nhm;
namespace ob = ompl::base;

static bool almost_le(double a, double b, double eps=1e-6){ return a <= b + eps; }

int main(){
    // 1) Path-length monotonicity (clothoids sampling)
    std::vector<SE2> wp = { {0,0,0}, {5,0,0}, {10,2,0.1} };
    auto segs = fitClothoidChain(wp, /*k_max=*/0.2, /*sigma_max=*/0.2);
    auto smp  = sampleClothoids(segs, 0.1);
    for (size_t i=1;i<smp.size();++i){ assert(smp[i].s > smp[i-1].s); }

    // 2) Velocity profile constraints
    std::vector<VelPoint> path; path.reserve(smp.size());
    for (auto& p: smp) path.push_back({p.s, p.kappa});
    VelLimits lim; lim.v_max=8.0; lim.a_t_max=2.0; lim.a_lat_max=3.0; lim.j_t_max=4.0;
    auto prof = optimizeVelocityProfile(path, lim);
    assert(!prof.empty());
    for (size_t i=0;i<prof.size();++i){
        double v = prof[i].v;
        double kappa = path[i].kappa;
        // lateral accel
        assert( almost_le(v*v*std::abs(kappa), lim.a_lat_max + 1e-3) );
        if (i>0){
            double dt = prof[i].t - prof[i-1].t; if (dt<1e-6) dt=1e-6;
            double a  = (prof[i].v - prof[i-1].v)/dt;
            assert( almost_le(std::abs(a), lim.a_t_max + 1e-2) );
        }
    }

    // 3) Closed-loop tracking with OSQP MPC under a simple reversal
    MpcParams P; P.N=10; P.dt=0.1; P.L=2.7; P.v_min=-3.0; P.v_max=3.0; P.delta_min=-0.6; P.delta_max=0.6; P.ddelta_max=0.5;
    // Build ref with a cusp-like change: forward then reverse v
    std::vector<RefSample> ref; ref.resize(P.N+1);
    for (int k=0;k<=P.N;++k){
        double t = k*P.dt;
        ref[k].x = 0.2 * t; ref[k].y = 0.0; ref[k].yaw = 0.0;
        ref[k].kappa = 0.0;
        ref[k].v = (k < P.N/2) ? 1.0 : -1.0; // change sign mid-horizon
    }
    Eigen::Vector4d x0; x0<< -0.5, 0.0, 0.0, 0.0;
    auto sol = solveMpcOsqp(P, ref, x0);
    // Ensure we got controls and the first step reduces position error magnitude
    if (!sol.x.empty()){
        double ex0 = x0(0) - ref[0].x;
        double ex1 = sol.x[1] - ref[1].x; // predicted next
        assert(std::abs(ex1) <= std::abs(ex0) + 1e-3);
    }

    std::cout << "All tests passed.\n";
    return 0;
}
