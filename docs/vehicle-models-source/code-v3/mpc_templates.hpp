// mpc_templates.hpp
#pragma once
#include <vector>
#include <Eigen/Dense>

namespace nhm {

// Simple linearized bicycle MPC (discrete) for tracking (x,y,psi,v) to reference samples.
// Two backends: CASADI (nonlinear) if available; OSQP (linearized QP) always as fallback.

struct MpcParams {
    int N{20};          // horizon steps
    double dt{0.1};
    double L{2.7};
    double w_pos{2.0}, w_yaw{1.0}, w_v{0.1}, w_u{0.01}, w_du{0.1};
    double v_min{-3.0}, v_max{3.0};
    double delta_min{-0.6}, delta_max{0.6};
    double ddelta_max{0.5}; // rate
};

struct RefSample { double x,y,yaw,kappa,v; };

struct MpcSolution { std::vector<double> u_delta, u_a; std::vector<double> x, y, yaw, v; };

// OSQP-based linear MPC around a provided reference (A,B matrices per step are built by small-angle linearization).
MpcSolution solveMpcOsqp(const MpcParams& P,
                         const std::vector<RefSample>& ref,
                         const Eigen::Vector4d& x0);

#ifdef HAS_CASADI
// Nonlinear MPC using CasADi (if available); same interface.
MpcSolution solveMpcCasadi(const MpcParams& P,
                           const std::vector<RefSample>& ref,
                           const Eigen::Vector4d& x0);
#endif

} // namespace nhm
