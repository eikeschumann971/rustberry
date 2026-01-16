# Nonholonomic OMPL Scaffolding

This folder contains a compact C++ scaffolding to plan with **low-speed non-holonomic vehicles** in **OMPL's control layer**, plus **Dubins smoothing** and **Reeds–Shepp** geometric planning.

- **Kinematic Bicycle** (state `[SE2|δ]`, control `[v, dδ]`)
- **4WS / Dual Steering** (state `[SE2|δf, δr]`, control `[v, dδf, dδr]`)
- **Tractor + 1 Trailer (off-axle)** (state `[SE2|δ0, β1]`, control `[v0, dδ0]`)
- Planners (control): **KPIECE1**, **SST**
- Geometric helpers: **Dubins** space (for smoothing) and **Reeds–Shepp** space (for geometric planning)
- Tiny **JSON loader** for parameters and an **optional pybind11** stub

### Why these designs match OMPL and the literature
- Control planning requires **state + control spaces** and a **StatePropagator** / `ODESolver`.  
  See OMPL’s ODE tutorial and `StatePropagator` docs.
- KPIECE1 and SST are standard **kinodynamic** planners in OMPL.  
- Dubins/Reeds–Shepp state spaces provide **curvature‑constrained interpolation** used by `PathSimplifier` during smoothing.

---

## Build (CMake)

```cmake
cmake_minimum_required(VERSION 3.12)
project(nhm_ompl LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED)

add_library(nhm STATIC nonholonomic_models.cpp)
target_include_directories(nhm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(nhm PUBLIC ompl)

add_executable(example main.cpp)
target_link_libraries(example PRIVATE nhm)
```

**Example `main.cpp`**
```cpp
#include "nonholonomic_models.hpp"
#include <iostream>
int main(){
    using namespace nhm; namespace ob=ompl::base; namespace oc=ompl::control;
    BicycleParams P; ob::RealVectorBounds xy(2); xy.setLow(-20); xy.setHigh(20);
    auto B = createBicycleSetup(xy, P, PlannerType::KPIECE, 0.02, 1, 10);
    B.ss->setStateValidityChecker([](const ob::State*){return true;});
    ob::ScopedState<> s(B.space), g(B.space);
    auto* se2s = s->as<ob::SE2StateSpace::StateType>(0);
    auto* delS = s->as<ob::RealVectorStateSpace::StateType>(1);
    se2s->setX(0); se2s->setY(0); se2s->setYaw(0); (*delS)[0]=0.0;
    auto* se2g = g->as<ob::SE2StateSpace::StateType>(0);
    auto* delG = g->as<ob::RealVectorStateSpace::StateType>(1);
    se2g->setX(10); se2g->setY(5); se2g->setYaw(1.57); (*delG)[0]=0.0;
    B.ss->setStartAndGoalStates(s,g,0.5);
    B.ss->setup();
    if (B.ss->solve(2.0)) {
        auto pc = B.ss->getSolutionPath();
        // Dubins smoothing (requires turning radius)
        auto smoothed = nhm::dubinsSmoothFromControlPath(6.0, xy, B.ss->getSpaceInformation(), pc,
            [](const ob::State* st){ return true; }, 200, 1.0, true);
        std::cout << "Smoothed with " << smoothed.getStateCount() << " states\n";
    } else std::cout << "No solution" << std::endl;
}
```

Build and run:
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
./example
```

---

## Dubins smoothing & Reeds–Shepp helpers

### Dubins smoothing from a control path
```cpp
auto smoothed = nhm::dubinsSmoothFromControlPath(
    /*turningRadius=*/6.0,
    /*xyBounds=*/xy,
    /*ctrlSI=*/B.ss->getSpaceInformation(),
    /*ctrlPath=*/B.ss->getSolutionPath(),
    /*se2Validity=*/[](const ob::State* s){ return true; },
    /*shortcutIters=*/200,
    /*maxTime=*/1.0,
    /*reduceVerts=*/true);
```
This converts the **control** path to **SE2**, switches to a **DubinsStateSpace**, and runs OMPL’s
**PathSimplifier**. Since the state space interpolation is **Dubins**, shortcutting preserves the
curvature bound. (See OMPL Dubins state space docs and `PathGeometric` / `PathSimplifier` APIs.)

### One‑shot Reeds–Shepp planner
```cpp
auto rsPath = nhm::planReedsSheppSimple(
    /*xyBounds=*/xy, /*turningRadius=*/6.0,
    /*validity=*/[](const ob::State* s){ return true; },
    /*startSE2=*/{0.0,0.0,0.0},
    /*goalSE2=*/{10.0,5.0,1.57},
    /*timeSeconds=*/2.0,
    /*plannerName=*/"RRTConnect");
```
This uses OMPL’s **ReedsSheppStateSpace** with either **RRTConnect** (fast) or **RRTstar** (optimal) and
returns a `PathGeometric`. You can further shorten it with `PathSimplifier` which respects RS interpolation.

---

## Optional: pybind11 module
```cmake
find_package(pybind11 REQUIRED)
pybind11_add_module(nhm_bindings bindings_pybind11.cpp nonholonomic_models.cpp)
target_link_libraries(nhm_bindings PRIVATE ompl)
```
Python usage:
```python
import nhm_bindings as nhm
traj_rs = nhm.plan_reeds_shepp_example(0,0,0, 10,5,1.57, 6.0, 2.0, "RRTConnect")
print(len(traj_rs), 'states')
```
