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
    // Simple heuristic: set length L by chord, linearly ramp kappa from k0 to k1, bound by limits
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
    // crude curvature guesses from heading changes over chord length
    for (size_t i=0;i+1<wp.size();++i){
        const auto& A=wp[i]; const auto& B=wp[i+1];
        double dx=B.x-A.x, dy=B.y-A.y; double L=std::hypot(dx,dy);
        double dyaw = normYaw(B.yaw - A.yaw);
        double k = (L>1e-6)? dyaw / L : 0.0; // average curvature over the segment
        double k0 = k, k1 = k; // use same at both ends (heuristic); could be refined from neighbors
        out.push_back(fitTwoPointG2(A,B,k0,k1,k_max,sigma_max));
    }
    return out;
}

static void evalClothoid(const ClothoidSeg& c, double s, double& x, double& y, double& yaw, double& kappa){
    kappa = c.k0 + c.sigma*s;
    yaw = c.yaw0 + c.k0*s + 0.5*c.sigma*s*s;
    // simple tangent integration (Euler) with small step: acceptable for small ds in sampling
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


std::pair<double,double> trailerAwareKappaSigmaBounds(double L0, double L1, double a_off,
                                                     double beta_max,
                                                     double v_ref,
                                                     double a_lat_max){
    // Lateral acceleration cap for tractor:
    double k_lat = (v_ref>1e-3) ? (a_lat_max/(v_ref*v_ref)) : 0.2;
    // Keep trailer articulation within beta_max: bound tractor curvature using sin(beta_max)/L1
    double k_beta = std::max(1e-6, std::sin(std::min(std::abs(beta_max), 1.4)) / std::max(1e-6, L1));
    double k_max = std::min(k_lat, k_beta);
    // Conservative curvature‑rate bound proportional to k_max and wheelbase
    double sigma_max = k_max / std::max(1.0, L0);
    return {k_max, sigma_max};
}

} // namespace nhm
