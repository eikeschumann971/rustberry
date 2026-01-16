# Nonholonomic OMPL Scaffolding

This folder contains a compact C++ scaffolding to plan with **low-speed non-holonomic vehicles** in **OMPL's control layer**:

- **Kinematic Bicycle** (state `[SE2|δ]`, control `[v, dδ]`)
- **4WS / Dual Steering** (state `[SE2|δf, δr]`, control `[v, dδf, dδr]`)
- **Tractor + 1 Trailer (off-axle)** (state `[SE2|δ0, β1]`, control `[v0, dδ0]`)
- Planners: **KPIECE1** and **SST** (control/kinodynamic planners)
- A tiny **JSON loader** for parameters (no external deps), and an **optional pybind11 stub**.

### Why these designs match OMPL and the literature
- OMPL control-based planning expects a **state space**, **control space**, and a **StatePropagator** (or `ODESolver`) to integrate ODEs. See OMPL’s **Using the ODESolver** tutorial and `StatePropagator` docs.  
  <https://ompl.kavrakilab.org/odeint.html> · <https://ompl.kavrakilab.org/classompl_1_1control_1_1StatePropagator.html>
- **KPIECE1** and **SST** are documented control planners suitable for kinodynamic problems.  
  <https://ompl.kavrakilab.org/classompl_1_1control_1_1KPIECE1.html> · <https://docs.ros.org/en/rolling/p/ompl/generated/classompl_1_1control_1_1SST.html>
- The **4WS curvature** formula and **Ackermann** relations are consistent with MathWorks’ Vehicle Dynamics Blockset docs.  
  <https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html>
- The **tractor–trailer kinematics** with off-axle hitch follow van de Wouw et al. (CDC 2015).  
  <https://vandewouw.dc.tue.nl/CDC2015_vandeWouw_Ritzen.pdf>

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
        auto& path=B.ss->getSolutionPath(); path.interpolate();
        std::cout << "Solved with " << path.getStateCount() << " states\n";
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

## Optional: pybind11 module
If you want Python bindings:
```cmake
find_package(pybind11 REQUIRED)
pybind11_add_module(nhm_bindings bindings_pybind11.cpp nonholonomic_models.cpp)
target_link_libraries(nhm_bindings PRIVATE ompl)
```
Usage in Python:
```python
import nhm_bindings as nhm
traj = nhm.plan_bicycle_example(0,0,0, 10,5,1.57, 2.7, 0.6, 2.0)
print(len(traj), 'states')
```

## Notes
- Replace the trivial `StateValidityChecker` with your collision checker (FCL/Octomap, etc.).
- For **4WS** and **tractor–trailer**, call `createFourWSSetup` / `createTractorTrailerSetup` and populate start/goal accordingly.
- If you prefer **ODEINT** integration, swap the propagators for `ompl::control::ODESolver` (see tutorial).  
  <https://ompl.kavrakilab.org/odeint.html>
```
