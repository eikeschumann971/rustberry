// nonholonomic_models.hpp
// Minimal OMPL-based scaffolding for low-speed non-holonomic vehicles
// Models: Kinematic Bicycle, 4WS (dual steering), Tractor + 1 Trailer (off-axle)
// Planners: KPIECE1, SST (control-based) + Reeds–Shepp (geometric)
// Utilities: Dubins smoothing (PathSimplifier) + tiny JSON params loader
//
// References used to match OMPL APIs and model equations:
// - OMPL ODESolver & control-based planning: https://ompl.kavrakilab.org/odeint.html
// - StatePropagator: https://ompl.kavrakilab.org/classompl_1_1control_1_1StatePropagator.html
// - KPIECE1: https://ompl.kavrakilab.org/classompl_1_1control_1_1KPIECE1.html
// - SST (kinodynamic): https://docs.ros.org/en/rolling/p/ompl/generated/classompl_1_1control_1_1SST.html
// - DubinsStateSpace / ReedsSheppStateSpace docs: OMPL C++ API
// - 4WS kinematics: MathWorks Vehicle Dynamics Blockset (Kinematic Steering)
// - Tractor–trailer low-speed kinematics (off-axle): van de Wouw et al., CDC 2015

#pragma once

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/sst/SST.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <array>
#include <functional>

namespace nhm {

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

// --------------------------- Parameter structs ---------------------------
struct BicycleParams {
    double L{2.7};
    double vMin{-2.0}, vMax{2.0};
    double dMin{-0.6}, dMax{0.6};
    double ddMin{-0.5}, ddMax{0.5};
    bool   clampSteer{true};
    bool   loadFromJson(const std::string& path);
};

struct FourWSParams {
    double L{2.9};
    double vMin{-2.0}, vMax{2.0};
    double dfMin{-0.6}, dfMax{0.6};
    double drMin{-0.6}, drMax{0.6};
    double ddfMin{-0.5}, ddfMax{0.5};
    double ddrMin{-0.5}, ddrMax{0.5};
    bool   clampSteer{true};
    bool   loadFromJson(const std::string& path);
};

struct TrailerParams {
    double L0{3.2}, L1{6.0}, a{1.0};
    double vMin{-1.5}, vMax{1.5};
    double d0Min{-0.6}, d0Max{0.6};
    double dd0Min{-0.4}, dd0Max{0.4};
    double betaMin{-1.57}, betaMax{1.57};
    bool   clampSteer{true};
    bool   loadFromJson(const std::string& path);
};

// --------------------------- Model & planner enums ---------------------------
enum class ModelType { Bicycle, FourWS, TractorTrailer };
enum class PlannerType { KPIECE, SST };

// --------------------------- Builders (control) ---------------------------
// Bicycle: [SE2 | δ]
ob::StateSpacePtr buildBicycleStateSpace(const ob::RealVectorBounds& xyBounds,
                                         const BicycleParams& P);
oc::ControlSpacePtr buildBicycleControlSpace(const ob::StateSpacePtr& space,
                                            const BicycleParams& P);
// 4WS: [SE2 | δf, δr]
ob::StateSpacePtr buildFourWSStateSpace(const ob::RealVectorBounds& xyBounds,
                                        const FourWSParams& P);
oc::ControlSpacePtr buildFourWSControlSpace(const ob::StateSpacePtr& space,
                                            const FourWSParams& P);
// Tractor + 1 trailer: [SE2 | δ0, β1]
ob::StateSpacePtr buildTractorTrailerStateSpace(const ob::RealVectorBounds& xyBounds,
                                                const TrailerParams& P);
oc::ControlSpacePtr buildTractorTrailerControlSpace(const ob::StateSpacePtr& space,
                                                    const TrailerParams& P);

// Projections (for KPIECE/SST)
void registerDefaultProjections(const ob::StateSpacePtr& space, ModelType model);

// Propagators
std::shared_ptr<oc::StatePropagator> makeBicyclePropagator(const oc::SpaceInformationPtr& si,
                                                           BicycleParams P);
std::shared_ptr<oc::StatePropagator> makeFourWSPropagator(const oc::SpaceInformationPtr& si,
                                                          FourWSParams P);
std::shared_ptr<oc::StatePropagator> makeTractorTrailerPropagator(const oc::SpaceInformationPtr& si,
                                                                  TrailerParams P);

// Planner factory (control)
ompl::base::PlannerPtr makePlanner(PlannerType which, const oc::SpaceInformationPtr& si);

// Convenience: one-shot control SimpleSetup per model
struct SetupBundle {
    oc::SimpleSetupPtr ss;
    ob::StateSpacePtr  space;
    oc::ControlSpacePtr control;
};

SetupBundle createBicycleSetup(const ob::RealVectorBounds& xy, const BicycleParams& P,
                               PlannerType planner, double dt = 0.02,
                               unsigned minSteps=1, unsigned maxSteps=10);
SetupBundle createFourWSSetup(const ob::RealVectorBounds& xy, const FourWSParams& P,
                              PlannerType planner, double dt = 0.02,
                              unsigned minSteps=1, unsigned maxSteps=10);
SetupBundle createTractorTrailerSetup(const ob::RealVectorBounds& xy, const TrailerParams& P,
                                      PlannerType planner, double dt = 0.02,
                                      unsigned minSteps=1, unsigned maxSteps=10);

// --------------------------- Geometric (Dubins / Reeds–Shepp) ---------------------------
// State spaces
ob::StateSpacePtr makeDubinsSpace(double turningRadius, const ob::RealVectorBounds& xyBounds, bool symmetric=false);
ob::StateSpacePtr makeReedsSheppSpace(double turningRadius, const ob::RealVectorBounds& xyBounds);

// Simple setups
std::shared_ptr<og::SimpleSetup> createDubinsSetup(double turningRadius, const ob::RealVectorBounds& xyBounds);
std::shared_ptr<og::SimpleSetup> createReedsSheppSetup(double turningRadius, const ob::RealVectorBounds& xyBounds);

// Dubins smoothing for a geometric path
og::PathGeometric dubinsSmooth(const std::shared_ptr<og::SimpleSetup>& ss,
                               const og::PathGeometric& in,
                               unsigned shortcutIters=200,
                               double maxTime=1.0,
                               bool reduceVerts=true);

// Convert a control path to SE2 and smooth with Dubins constraints
og::PathGeometric dubinsSmoothFromControlPath(double turningRadius,
                                              const ob::RealVectorBounds& xyBounds,
                                              const oc::SpaceInformationPtr& ctrlSI,
                                              const oc::PathControl& ctrlPath,
                                              const std::function<bool(const ob::State*)>& se2Validity,
                                              unsigned shortcutIters=200,
                                              double maxTime=1.0,
                                              bool reduceVerts=true);

// Convenience: one-shot Reeds–Shepp planning
og::PathGeometric planReedsSheppSimple(const ob::RealVectorBounds& xyBounds,
                                       double turningRadius,
                                       const std::function<bool(const ob::State*)>& validity,
                                       const std::array<double,3>& startSE2,
                                       const std::array<double,3>& goalSE2,
                                       double timeSeconds=1.0,
                                       const std::string& plannerName="RRTConnect");

// --------------------------- Tiny JSON helper (fallback) ---------------------------
std::unordered_map<std::string, double> loadJsonNumbers(const std::string& path);

} // namespace nhm
