// bindings_pybind11.cpp
// Optional: pybind11 stub to expose minimal planning entry points.
// To build, define -DNHM_PYBIND and have pybind11 available in your include path.
// Example CMake:
//   find_package(pybind11 REQUIRED)
//   pybind11_add_module(nhm_bindings bindings_pybind11.cpp nonholonomic_models.cpp)
//   target_link_libraries(nhm_bindings PRIVATE ompl)
//
// OMPL control design & StatePropagator usage matches:
// - ODESolver/controls tutorial: https://ompl.kavrakilab.org/odeint.html
// - StatePropagator: https://ompl.kavrakilab.org/classompl_1_1control_1_1StatePropagator.html

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "nonholonomic_models.hpp"

namespace py = pybind11;
using nhm::PlannerType; using nhm::ModelType;

static std::vector<std::array<double,4>> plan_bicycle_example(double x0,double y0,double yaw0,
                                                              double xg,double yg,double yawg,
                                                              double L, double turnMax,
                                                              double solveSeconds=2.0) {
    nhm::BicycleParams P; P.L=L; P.dMin=-turnMax; P.dMax=turnMax;
    ompl::base::RealVectorBounds xy(2); xy.setLow(-50); xy.setHigh(50);
    auto B = nhm::createBicycleSetup(xy, P, PlannerType::KPIECE, 0.02, 1, 10);
    // trivial validity (user should replace)
    B.ss->setStateValidityChecker([](const ompl::base::State*){ return true; });

    ompl::base::ScopedState<> s(B.space), g(B.space);
    // Compound: [SE2 | δ]
    auto* Sse2 = s->as<ompl::base::SE2StateSpace::StateType>(0);
    auto* Sdel = s->as<ompl::base::RealVectorStateSpace::StateType>(1);
    Sse2->setX(x0); Sse2->setY(y0); Sse2->setYaw(yaw0); (*Sdel)[0]=0.0;
    auto* Gse2 = g->as<ompl::base::SE2StateSpace::StateType>(0);
    auto* Gdel = g->as<ompl::base::RealVectorStateSpace::StateType>(1);
    Gse2->setX(xg); Gse2->setY(yg); Gse2->setYaw(yawg); (*Gdel)[0]=0.0;

    B.ss->setStartAndGoalStates(s, g, 0.5);
    B.ss->setup();
    if (!B.ss->solve(solveSeconds)) return {};

    std::vector<std::array<double,4>> out; // x,y,yaw,delta
    auto& path = B.ss->getSolutionPath(); path.asGeometric(); // ensure interpolation ok
    path.interpolate();

    // Extract states along path (controls available too if needed)
    for (std::size_t i=0;i<path.getStateCount();++i){
        const auto* st = path.getState(i)->as<ompl::base::CompoundState>();
        const auto* se2 = st->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto* sd  = st->as<ompl::base::RealVectorStateSpace::StateType>(1);
        out.push_back({se2->getX(), se2->getY(), se2->getYaw(), (*sd)[0]});
    }
    return out;
}

PYBIND11_MODULE(nhm_bindings, m) {
    m.doc() = "Non-holonomic low-speed planners (OMPL control)";
    py::enum_<PlannerType>(m, "PlannerType")
        .value("KPIECE", PlannerType::KPIECE)
        .value("SST", PlannerType::SST);
    m.def("plan_bicycle_example", &plan_bicycle_example,
          py::arg("x0"), py::arg("y0"), py::arg("yaw0"),
          py::arg("xg"), py::arg("yg"), py::arg("yawg"),
          py::arg("L")=2.7, py::arg("turnMax")=0.6, py::arg("solveSeconds")=2.0,
          R"pbdoc(Returns a list of [x,y,yaw,delta] along the planned path using KPIECE.
This is a minimal stub; replace validity checker and space bounds for your environment.)pbdoc");
}
