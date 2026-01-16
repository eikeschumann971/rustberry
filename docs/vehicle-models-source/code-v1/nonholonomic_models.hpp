// nonholonomic_models.hpp
// Minimal OMPL-based scaffolding for low-speed non-holonomic vehicles
// Models: Kinematic Bicycle, 4WS (dual steering), Tractor + 1 Trailer (off-axle)
// Planners: KPIECE1, SST (control-based)
// JSON params loader (header-only fallback via simple regex parsing; see .cpp)
//
// References used to match OMPL APIs and model equations:
// - OMPL control tutorials & ODESolver: https://ompl.kavrakilab.org/odeint.html
// - OMPL StatePropagator API: https://ompl.kavrakilab.org/classompl_1_1control_1_1StatePropagator.html
// - KPIECE1 (control): https://ompl.kavrakilab.org/classompl_1_1control_1_1KPIECE1.html
// - SST (control, kinodynamic): https://docs.ros.org/en/rolling/p/ompl/generated/classompl_1_1control_1_1SST.html
// - Kinematic steering relations (4WS): MathWorks Vehicle Dynamics Blockset, Kinematic Steering
//   https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
// - Tractor–trailer low-speed kinematics with off-axle hitch: van de Wouw et al., CDC 2015
//   https://vandewouw.dc.tue.nl/CDC2015_vandeWouw_Ritzen.pdf

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

#include <memory>
#include <string>
#include <unordered_map>

namespace nhm {

namespace ob = ompl::base;
namespace oc = ompl::control;

// --------------------------- Parameter structs ---------------------------
struct BicycleParams {
    double L{2.7};
    double vMin{-2.0}, vMax{2.0};
    double dMin{-0.6}, dMax{0.6};
    double ddMin{-0.5}, ddMax{0.5};
    bool   clampSteer{true};
    bool   loadFromJson(const std::string& path); // implemented in .cpp
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

// --------------------------- Builders ---------------------------
// All builders return a compound state space appropriate for each model
// Bicycle: [SE2 | δ]
ob::StateSpacePtr buildBicycleStateSpace(const ob::RealVectorBounds& xyBounds,
                                         const BicycleParams& P);
// Control space: [v, dδ]
oc::ControlSpacePtr buildBicycleControlSpace(const ob::StateSpacePtr& space,
                                            const BicycleParams& P);

// 4WS: [SE2 | δf, δr]
ob::StateSpacePtr buildFourWSStateSpace(const ob::RealVectorBounds& xyBounds,
                                        const FourWSParams& P);
oc::ControlSpacePtr buildFourWSControlSpace(const ob::StateSpacePtr& space,
                                            const FourWSParams& P);

// Tractor + single trailer: [SE2 | δ0, β1]
ob::StateSpacePtr buildTractorTrailerStateSpace(const ob::RealVectorBounds& xyBounds,
                                                const TrailerParams& P);
oc::ControlSpacePtr buildTractorTrailerControlSpace(const ob::StateSpacePtr& space,
                                                    const TrailerParams& P);

// --------------------------- Projections ---------------------------
// Registers model-appropriate projections for KPIECE/SST use.
void registerDefaultProjections(const ob::StateSpacePtr& space, ModelType model);

// --------------------------- Propagators ---------------------------
std::shared_ptr<oc::StatePropagator> makeBicyclePropagator(const oc::SpaceInformationPtr& si,
                                                           BicycleParams P);
std::shared_ptr<oc::StatePropagator> makeFourWSPropagator(const oc::SpaceInformationPtr& si,
                                                          FourWSParams P);
std::shared_ptr<oc::StatePropagator> makeTractorTrailerPropagator(const oc::SpaceInformationPtr& si,
                                                                  TrailerParams P);

// --------------------------- Planner factory ---------------------------
ompl::base::PlannerPtr makePlanner(PlannerType which, const oc::SpaceInformationPtr& si);

// --------------------------- Convenience: one-shot setup ---------------------------
// Create a control::SimpleSetup for a chosen model and planner.
// Caller sets start/goal and calls solve().
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

// --------------------------- Tiny JSON helper (fallback) ---------------------------
// A very small loader that maps string->double for flat JSON files.
// Accepts numbers and booleans (as 0/1); ignores nesting and arrays.
// This is a simplistic helper so you can avoid external deps if desired.
std::unordered_map<std::string, double> loadJsonNumbers(const std::string& path);

} // namespace nhm
