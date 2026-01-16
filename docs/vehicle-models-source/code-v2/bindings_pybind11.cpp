// bindings_pybind11.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "nonholonomic_models.hpp"

namespace py = pybind11;
using nhm::PlannerType;

static std::vector<std::array<double,4>> plan_bicycle_example(double x0,double y0,double yaw0,
                                                              double xg,double yg,double yawg,
                                                              double L, double turnMax,
                                                              double solveSeconds=2.0) {
    nhm::BicycleParams P; P.L=L; P.dMin=-turnMax; P.dMax=turnMax;
    ompl::base::RealVectorBounds xy(2); xy.setLow(-50); xy.setHigh(50);
    auto B = nhm::createBicycleSetup(xy, P, PlannerType::KPIECE, 0.02, 1, 10);
    B.ss->setStateValidityChecker([](const ompl::base::State*){ return true; });

    ompl::base::ScopedState<> s(B.space), g(B.space);
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
    auto geo = B.ss->getSolutionPath().asGeometric();
    geo.interpolate();
    for (std::size_t i=0;i<geo.getStateCount();++i){
        const auto* st = geo.getState(i)->as<ompl::base::CompoundState>();
        const auto* se2 = st->as<ompl::base::SE2StateSpace::StateType>(0);
        const auto* sd  = st->as<ompl::base::RealVectorStateSpace::StateType>(1);
        out.push_back({se2->getX(), se2->getY(), se2->getYaw(), (*sd)[0]});
    }
    return out;
}

static std::vector<std::array<double,3>> plan_reeds_shepp_example(double x0,double y0,double yaw0,
                                                                  double xg,double yg,double yawg,
                                                                  double turningRadius,
                                                                  double solveSeconds=2.0,
                                                                  const std::string &plannerName="RRTConnect") {
    ompl::base::RealVectorBounds xy(2); xy.setLow(-50); xy.setHigh(50);
    auto path = nhm::planReedsSheppSimple(xy, turningRadius,
        [](const ompl::base::State*){ return true; },
        std::array<double,3>{x0,y0,yaw0}, std::array<double,3>{xg,yg,yawg}, solveSeconds, plannerName);
    std::vector<std::array<double,3>> out;
    for (std::size_t i=0;i<path.getStateCount();++i){
        const auto* se2 = path.getState(i)->as<ompl::base::SE2StateSpace::StateType>();
        out.push_back({se2->getX(), se2->getY(), se2->getYaw()});
    }
    return out;
}

PYBIND11_MODULE(nhm_bindings, m) {
    m.doc() = "Non-holonomic low-speed planners (OMPL control + geometric)";
    py::enum_<PlannerType>(m, "PlannerType")
        .value("KPIECE", PlannerType::KPIECE)
        .value("SST", PlannerType::SST);
    m.def("plan_bicycle_example", &plan_bicycle_example,
          py::arg("x0"), py::arg("y0"), py::arg("yaw0"),
          py::arg("xg"), py::arg("yg"), py::arg("yawg"),
          py::arg("L")=2.7, py::arg("turnMax")=0.6, py::arg("solveSeconds")=2.0);
    m.def("plan_reeds_shepp_example", &plan_reeds_shepp_example,
          py::arg("x0"), py::arg("y0"), py::arg("yaw0"),
          py::arg("xg"), py::arg("yg"), py::arg("yawg"),
          py::arg("turningRadius")=6.0, py::arg("solveSeconds")=2.0, py::arg("plannerName")="RRTConnect");
}
