// clothoid.cpp
#include "clothoid.hpp"
#include <cmath>
#include <algorithm>

namespace nhm {

static inline double normYaw(double a){ while (a>M_PI) a-=2*M_PI; while (a<-M_PI) a+=2*M_PI; return a; }

static ClothoidSeg fitTwoPointG2(const SE2& a, const SE2& b,
                                 double k0_guess,
                                 double k1_guess,
                                 double k_max, double sigma_max){
    double dx=b.x-a.x, dy=b.y-a.y;
    double L = std::hypot(dx,dy);
    double k0 = std::clamp(k0_guess, -k_max, k_max);
    double k1 = std::clamp(k1_guess, -k_max, k_max);
    double sigma = (k1-k0)/std::max(1e-6, L);
    sigma = std::clamp(sigma, -sigma_max, sigma_max);
    return {a.x,a.y,a.yaw,k0,sigma,L};
}

std::vector<ClothoidSeg> fitClothoidChain(const std::vector<SE2>& wp,
                                          double k_max,
                                          double sigma_max){
    std::vector<ClothoidSeg> out;
    if (wp.size()<2) return out;
    for (size_t i=0;i+1<wp.size();++i){
        const auto& A=wp[i]; const auto& B=wp[i+1];
        double dx=B.x-A.x, dy=B.y-A.y; double L=std::hypot(dx,dy);
        double dyaw = normYaw(B.yaw - A.yaw);
        double k = (L>1e-6)? dyaw / L : 0.0; // average curvature over the segment
        out.push_back(fitTwoPointG2(A,B,k,k,k_max,sigma_max));
    }
    return out;
}

static void evalClothoid(const ClothoidSeg& c, double s, double& x, double& y, double& yaw, double& kappa){
    kappa = c.k0 + c.sigma*s;
    yaw = c.yaw0 + c.k0*s + 0.5*c.sigma*s*s;
    x = c.x0 + s*std::cos(yaw);
    y = c.y0 + s*std::sin(yaw);
}

std::vector<ClothoidSample> sampleClothoids(const std::vector<ClothoidSeg>& segs, double ds){
    std::vector<ClothoidSample> out; if (segs.empty()) return out;
    double s_acc=0.0;
    for (const auto& c: segs){
        for (double s=0.0; s<c.L; s+=ds){
            double x,y,yaw,k; evalClothoid(c,s,x,y,yaw,k);
            out.push_back({s_acc+s,x,y,yaw,k});
        }
        double x,y,yaw,k; evalClothoid(c,c.L,x,y,yaw,k);
        out.push_back({s_acc+c.L,x,y,yaw,k});
        s_acc += c.L;
    }
    return out;
}

// Improved trailer-aware bound using beta_dot_max and steering rate
std::pair<double,double> trailerAwareKappaSigmaBounds(double L0, double L1, double a_off,
                                                     double beta_max,
                                                     double beta_dot_max,
                                                     double v_ref,
                                                     double a_lat_max,
                                                     double ddelta_max){
    // Lateral acceleration bound
    double k_lat = (v_ref>1e-3) ? (a_lat_max/(v_ref*v_ref)) : 0.2;
    // Geometry bound from articulation angle
    double k_beta = std::max(1e-6, std::sin(std::min(std::abs(beta_max), 1.4)) / std::max(1e-6, L1));
    // Articulation rate bound: |beta_dot| ≲ |v*kappa| + ... <= beta_dot_max → kappa ≤ beta_dot_max / v
    double k_bdot = (v_ref>1e-3) ? (beta_dot_max / v_ref) : k_lat;
    double k_max = std::min(k_lat, std::min(k_beta, k_bdot));

    // Sigma bound from steering-rate: k = tan(delta)/L0 ⇒ dot{k} ≈ (sec^2(delta)/L0)*dot{delta}; σ = dot{k}/v
    double sec2 = 1.0; // worst-case small delta
    double sigma_sr = (v_ref>1e-3) ? (sec2 * ddelta_max / (L0 * v_ref)) : (ddelta_max/(L0*1.0));
    // Combine with geometric conservative bound σ ≤ k_max / L0 (very conservative)
    double sigma_geom = k_max / std::max(1.0, L0);
    double sigma_max = std::min(sigma_sr, sigma_geom);
    return {k_max, sigma_max};
}

} // namespace nhm
