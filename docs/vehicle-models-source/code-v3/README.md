

---

## New modules

### 1) Time‑optimal velocity profile with jerk limits
Files: `velocity_profile.hpp/.cpp`

- **What it does**: discretized **TOPP** along path length `s` with curvature bounds. It performs a forward/backward pass for **tangential acceleration** limits and an optional iterative pass to enforce **jerk** limits (|Δa| ≤ j_max Δt). The lateral limit enforces v²|κ| ≤ a_lat_max.  
  Background: **TOPP-RA / reachability** ideas and jerk-constrained variants. citeturn10search104turn10search106turn10search107turn10search108turn10search109

- **Call**:
```cpp
std::vector<VelPoint> path; // fill with (s, kappa)
VelLimits lim; lim.v_max=12; lim.a_t_max=2.0; lim.a_lat_max=3.0; lim.j_t_max=4.0;
auto prof = nhm::optimizeVelocityProfile(path, lim);
```

### 2) **G²** Clothoid smoothing (curvature‑continuous)
Files: `clothoid.hpp/.cpp`

- **What it does**: fits a **chain of clothoids** (linear curvature segments) between SE(2) waypoints, returning segments and dense samples. This produces **curvature‑continuous** (G²) paths better suited to jerk‑limited tracking than piecewise circular arcs.  
  References: robust **clothoid fitting** (Bertolazzi & Frego) and background on continuous‑curvature planning. citeturn10search92turn10search97turn10search96turn10search126

- **Call**:
```cpp
std::vector<nhm::SE2> wp = {{x0,y0,yaw0}, {x1,y1,yaw1}, ...};
auto segs = nhm::fitClothoidChain(wp, /*k_max=*/0.2, /*sigma_max=*/0.2);
auto samples = nhm::sampleClothoids(segs, /*ds=*/0.1);
```

### 3) **MPC templates** (CasADi / OSQP)
Files: `mpc_templates.hpp/.cpp`

- **What it does**: a minimal **linear MPC (OSQP)** template for the bicycle model and a **CasADi** hook (ifdef `HAS_CASADI`) to upgrade to NMPC.  
  See **CasADi** docs for building C++ optimal control/NLPs; **OSQP** docs and paper for fast embedded QPs. citeturn10search116turn10search121turn10search98turn10search101

- **Call**:
```cpp
nhm::MpcParams P; P.N=20; P.dt=0.1; P.L=2.7; 
std::vector<nhm::RefSample> ref = /* from RS/clothoid + velocity profile */;
Eigen::Vector4d x0(x, y, yaw, v);
auto sol = nhm::solveMpcOsqp(P, ref, x0); // or solveMpcCasadi if available
```

### 4) **Reverse‑aware** Reeds–Shepp tracking
Files: `rs_tracking.hpp/.cpp`

- **What it does**: converts a geometric RS path to a signed‑velocity reference for tracking (forward/reverse segments), inserting a gear flip at cusps.  
  Background: **Reeds–Shepp** shortest paths (forward+reverse) and common classifications; LaValle’s summary. citeturn10search122turn10search125

- **Call**:
```cpp
std::vector<double> xs, ys, yaws; // from RS path
auto ref = nhm::makeReverseAwareReference(xs,ys,yaws, /*R=*/6.0,
                                          /*v_fwd=*/3.0, /*v_rev=*/1.5);
```

---

## Suggested pipeline (global→local)
1. **Plan** with `planReedsSheppSimple` (geometric) to get SE(2) path. citeturn6search67
2. **G² smooth** with `fitClothoidChain` → `sampleClothoids` for curvature continuity. citeturn10search92
3. **Time‑parameterize** with `optimizeVelocityProfile` using lateral (v²|κ|) + (a, jerk) limits. citeturn10search104turn10search106
4. **Build reference** `(x,y,ψ,κ,v,t)` and pass to **MPC** (OSQP or CasADi). citeturn10search98turn10search101turn10search116

### Trailer‑aware notes
For tractor–trailer, keep |β| within bounds using your earlier kinematics; bound tractor curvature κ₀ and curvature rate σ so that induced β dynamics remain within `[β_min, β_max]` (see low‑speed off‑axle relations). The **clothoid** chain’s `k_max` and `sigma_max` act as direct knobs; tune them using wheelbase/hitch geometry from your model. citeturn6search50

---

## Build updates
Add to your CMake:
```cmake
add_library(nhm_planning STATIC 
  nonholonomic_models.cpp
  velocity_profile.cpp
  clothoid.cpp
  mpc_templates.cpp
  rs_tracking.cpp)

target_include_directories(nhm_planning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OSQP QUIET) # or vendor OSQP
if(OSQP_FOUND)
  target_link_libraries(nhm_planning PUBLIC osqp)
endif()
```

For **CasADi** NMPC, define `-DHAS_CASADI` and link CasADi per the docs. citeturn10search116
