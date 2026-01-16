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
        auto smoothed = nhm::dubinsSmoothFromControlPath(6.0, xy, B.ss->getSpaceInformation(), pc,
            [](const ob::State* st){ return true; }, 200, 1.0, true);
        std::cout << "Smoothed with " << smoothed.getStateCount() << " states\n";
    } else std::cout << "No solution" << std::endl;
}
