// nonholonomic_models.cpp
#include "nonholonomic_models.hpp"
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <fstream>
#include <regex>
#include <sstream>
#include <algorithm>

namespace nhm {

// --------------------------- Tiny JSON number loader ---------------------------
std::unordered_map<std::string, double> loadJsonNumbers(const std::string& path) {
    std::unordered_map<std::string, double> kv;
    std::ifstream f(path);
    if (!f) return kv;
    std::stringstream buffer; buffer << f.rdbuf();
    std::string s = buffer.str();
    // Very small regex: captures "key" : number (incl. scientific), true/false to 1/0
    std::regex pair(R"REGEX(\"([^\"]+)\"\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?|true|false))REGEX");
    std::smatch m; auto it = s.cbegin();
    while (std::regex_search(it, s.cend(), m, pair)) {
        std::string key = m[1].str();
        std::string val = m[2].str();
        std::transform(val.begin(), val.end(), val.begin(), ::tolower);
        double d = (val=="true")?1.0 : (val=="false"?0.0 : std::stod(val));
        kv[key]=d; it = m.suffix().first;
    }
    return kv;
}

// --------------------------- Params loadFromJson ---------------------------
bool BicycleParams::loadFromJson(const std::string& path) {
    auto kv = loadJsonNumbers(path); if (kv.empty()) return false;
    if (kv.count("L")) L = kv["L"]; if (kv.count("vMin")) vMin = kv["vMin"]; if (kv.count("vMax")) vMax = kv["vMax"];
    if (kv.count("dMin")) dMin = kv["dMin"]; if (kv.count("dMax")) dMax = kv["dMax"];
    if (kv.count("ddMin")) ddMin = kv["ddMin"]; if (kv.count("ddMax")) ddMax = kv["ddMax"];
    if (kv.count("clampSteer")) clampSteer = (kv["clampSteer"]!=0.0);
    return true;
}

bool FourWSParams::loadFromJson(const std::string& path) {
    auto kv = loadJsonNumbers(path); if (kv.empty()) return false;
    if (kv.count("L")) L = kv["L"]; if (kv.count("vMin")) vMin = kv["vMin"]; if (kv.count("vMax")) vMax = kv["vMax"];
    if (kv.count("dfMin")) dfMin = kv["dfMin"]; if (kv.count("dfMax")) dfMax = kv["dfMax"];
    if (kv.count("drMin")) drMin = kv["drMin"]; if (kv.count("drMax")) drMax = kv["drMax"];
    if (kv.count("ddfMin")) ddfMin = kv["ddfMin"]; if (kv.count("ddfMax")) ddfMax = kv["ddfMax"];
    if (kv.count("ddrMin")) ddrMin = kv["ddrMin"]; if (kv.count("ddrMax")) ddrMax = kv["ddrMax"];
    if (kv.count("clampSteer")) clampSteer = (kv["clampSteer"]!=0.0);
    return true;
}

bool TrailerParams::loadFromJson(const std::string& path) {
    auto kv = loadJsonNumbers(path); if (kv.empty()) return false;
    if (kv.count("L0")) L0 = kv["L0"]; if (kv.count("L1")) L1 = kv["L1"]; if (kv.count("a")) a = kv["a"];
    if (kv.count("vMin")) vMin = kv["vMin"]; if (kv.count("vMax")) vMax = kv["vMax"];
    if (kv.count("d0Min")) d0Min = kv["d0Min"]; if (kv.count("d0Max")) d0Max = kv["d0Max"];
    if (kv.count("dd0Min")) dd0Min = kv["dd0Min"]; if (kv.count("dd0Max")) dd0Max = kv["dd0Max"];
    if (kv.count("betaMin")) betaMin = kv["betaMin"]; if (kv.count("betaMax")) betaMax = kv["betaMax"];
    if (kv.count("clampSteer")) clampSteer = (kv["clampSteer"]!=0.0);
    return true;
}

// --------------------------- State spaces ---------------------------
ob::StateSpacePtr buildBicycleStateSpace(const ob::RealVectorBounds& xyBounds,
                                         const BicycleParams& P) {
    auto se2 = std::make_shared<ob::SE2StateSpace>(); se2->setBounds(xyBounds);
    auto steer = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds sb(1); sb.setLow(P.dMin); sb.setHigh(P.dMax); steer->setBounds(sb);
    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(se2, 1.0);
    space->addSubspace(steer, 0.1);
    space->lock();
    return space;
}

oc::ControlSpacePtr buildBicycleControlSpace(const ob::StateSpacePtr& space,
                                            const BicycleParams& P) {
    auto cs = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds cb(2);
    cb.setLow(0, P.vMin); cb.setHigh(0, P.vMax);
    cb.setLow(1, P.ddMin); cb.setHigh(1, P.ddMax);
    cs->setBounds(cb);
    return cs;
}

ob::StateSpacePtr buildFourWSStateSpace(const ob::RealVectorBounds& xyBounds,
                                        const FourWSParams& P) {
    auto se2 = std::make_shared<ob::SE2StateSpace>(); se2->setBounds(xyBounds);
    auto steer = std::make_shared<ob::RealVectorStateSpace>(2); // δf, δr
    ob::RealVectorBounds sb(2);
    sb.setLow(0, P.dfMin); sb.setHigh(0, P.dfMax);
    sb.setLow(1, P.drMin); sb.setHigh(1, P.drMax);
    steer->setBounds(sb);
    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(se2, 1.0);
    space->addSubspace(steer, 0.2);
    space->lock();
    return space;
}

oc::ControlSpacePtr buildFourWSControlSpace(const ob::StateSpacePtr& space,
                                            const FourWSParams& P) {
    auto cs = std::make_shared<oc::RealVectorControlSpace>(space, 3);
    ob::RealVectorBounds cb(3);
    cb.setLow(0, P.vMin);   cb.setHigh(0, P.vMax);
    cb.setLow(1, P.ddfMin); cb.setHigh(1, P.ddfMax);
    cb.setLow(2, P.ddrMin); cb.setHigh(2, P.ddrMax);
    cs->setBounds(cb);
    return cs;
}

ob::StateSpacePtr buildTractorTrailerStateSpace(const ob::RealVectorBounds& xyBounds,
                                                const TrailerParams& P) {
    auto se2 = std::make_shared<ob::SE2StateSpace>(); se2->setBounds(xyBounds);
    auto vec = std::make_shared<ob::RealVectorStateSpace>(2); // δ0, β1
    ob::RealVectorBounds vb(2);
    vb.setLow(0, P.d0Min); vb.setHigh(0, P.d0Max);
    vb.setLow(1, P.betaMin); vb.setHigh(1, P.betaMax);
    vec->setBounds(vb);
    auto space = std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(se2, 1.0);
    space->addSubspace(vec, 0.2);
    space->lock();
    return space;
}

oc::ControlSpacePtr buildTractorTrailerControlSpace(const ob::StateSpacePtr& space,
                                                    const TrailerParams& P) {
    auto cs = std::make_shared<oc::RealVectorControlSpace>(space, 2);
    ob::RealVectorBounds cb(2);
    cb.setLow(0, P.vMin); cb.setHigh(0, P.vMax);
    cb.setLow(1, P.dd0Min); cb.setHigh(1, P.dd0Max);
    cs->setBounds(cb);
    return cs;
}

// --------------------------- Projection evaluators ---------------------------
class XYProjection : public ob::ProjectionEvaluator {
  public:
    XYProjection(const ob::StateSpacePtr& space, unsigned steerDim = 0)
    : ob::ProjectionEvaluator(space), steerDim_(steerDim) {}
    unsigned int getDimension() const override { return 2 + (steerDim_ ? 1 : 0); }
    void project(const ob::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override {
        const auto* c = state->as<ob::CompoundState>();
        const auto* se2 = c->as<ob::SE2StateSpace::StateType>(0);
        projection[0] = se2->getX(); projection[1] = se2->getY();
        if (steerDim_){
            const auto* vec = c->as<ob::RealVectorStateSpace::StateType>(1);
            projection[2] = vec->values[steerDim_-1];
        }
    }
  private:
    unsigned steerDim_; // 0 -> none; 1 -> first steer component (e.g., β1)
};

void registerDefaultProjections(const ob::StateSpacePtr& space, ModelType model) {
    switch (model) {
        case ModelType::Bicycle: {
            space->registerProjection("xy", std::make_shared<XYProjection>(space));
            break;
        }
        case ModelType::FourWS: {
            space->registerProjection("xy", std::make_shared<XYProjection>(space));
            break;
        }
        case ModelType::TractorTrailer: {
            // project to [x, y, β1]
            space->registerProjection("xyb1", std::make_shared<XYProjection>(space, /*steerDim=*/2));
            break;
        }
    }
}

// --------------------------- Propagators ---------------------------
class BicycleProp : public oc::StatePropagator {
  public:
    BicycleProp(const oc::SpaceInformationPtr& si, BicycleParams p) : oc::StatePropagator(si), P_(p) {}
    void propagate(const ob::State* s, const oc::Control* u, double T, ob::State* out) const override {
        const auto* cs = s->as<ob::CompoundState>();
        const auto* se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto* sd  = cs->as<ob::RealVectorStateSpace::StateType>(1);
        double x=se2->getX(), y=se2->getY(), yaw=se2->getYaw();
        double d=sd->values[0];
        const double* uc = u->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double v=uc[0], dd=uc[1];
        double t=0, dt=si_->getPropagationStepSize();
        while (t < T) {
            x   += v*std::cos(yaw)*dt;
            y   += v*std::sin(yaw)*dt;
            yaw += (v/P_.L)*std::tan(d)*dt;
            d += dd*dt; if (P_.clampSteer) d = std::clamp(d, P_.dMin, P_.dMax);
            t += dt;
        }
        auto* r = out->as<ob::CompoundState>();
        auto* rse2 = r->as<ob::SE2StateSpace::StateType>(0);
        auto* rste = r->as<ob::RealVectorStateSpace::StateType>(1);
        rse2->setX(x); rse2->setY(y); rse2->setYaw(yaw); rste->values[0]=d;
        si_->getStateSpace()->enforceBounds(out);
    }
  private: BicycleParams P_;
};

class FourWSProp : public oc::StatePropagator {
  public:
    FourWSProp(const oc::SpaceInformationPtr& si, FourWSParams p) : oc::StatePropagator(si), P_(p) {}
    void propagate(const ob::State* s, const oc::Control* u, double T, ob::State* out) const override {
        const auto* cs = s->as<ob::CompoundState>();
        const auto* se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto* sig = cs->as<ob::RealVectorStateSpace::StateType>(1);
        double x=se2->getX(), y=se2->getY(), yaw=se2->getYaw();
        double df=sig->values[0], dr=sig->values[1];
        const double* uc = u->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double v=uc[0], ddf=uc[1], ddr=uc[2];
        double t=0, dt=si_->getPropagationStepSize();
        while (t < T) {
            double kappa = (std::tan(df) - std::tan(dr)) / P_.L; // MathWorks kinematic steering
            x   += v*std::cos(yaw)*dt;
            y   += v*std::sin(yaw)*dt;
            yaw += v*kappa*dt;
            df += ddf*dt; dr += ddr*dt;
            if (P_.clampSteer) {
                df = std::clamp(df, P_.dfMin, P_.dfMax);
                dr = std::clamp(dr, P_.drMin, P_.drMax);
            }
            t += dt;
        }
        auto* r = out->as<ob::CompoundState>();
        auto* rse2 = r->as<ob::SE2StateSpace::StateType>(0);
        auto* rvec = r->as<ob::RealVectorStateSpace::StateType>(1);
        rse2->setX(x); rse2->setY(y); rse2->setYaw(yaw);
        rvec->values[0]=df; rvec->values[1]=dr;
        si_->getStateSpace()->enforceBounds(out);
    }
  private: FourWSParams P_;
};

class TractorTrailerProp : public oc::StatePropagator {
  public:
    TractorTrailerProp(const oc::SpaceInformationPtr& si, TrailerParams p)
    : oc::StatePropagator(si), P_(p) {}
    void propagate(const ob::State* s, const oc::Control* u, double T, ob::State* out) const override {
        const auto* cs = s->as<ob::CompoundState>();
        const auto* se2 = cs->as<ob::SE2StateSpace::StateType>(0);
        const auto* vec = cs->as<ob::RealVectorStateSpace::StateType>(1); // [δ0, β1]
        double x=se2->getX(), y=se2->getY(), psi0=se2->getYaw();
        double d0=vec->values[0], beta=vec->values[1];
        const double* uc = u->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double v0=uc[0], dd0=uc[1];
        double t=0, dt=si_->getPropagationStepSize();
        while (t < T) {
            double psi0dot = (v0 / P_.L0) * std::tan(d0);
            x   += v0 * std::cos(psi0) * dt;
            y   += v0 * std::sin(psi0) * dt;
            psi0 += psi0dot * dt;
            d0   += dd0 * dt; if (P_.clampSteer) d0 = std::clamp(d0, P_.d0Min, P_.d0Max);
            double betadot = -psi0dot + (v0/P_.L1)*std::sin(beta)
                             + (P_.a * v0 / (P_.L0*P_.L1)) * std::tan(d0) * std::cos(beta);
            beta += betadot * dt; beta = std::clamp(beta, P_.betaMin, P_.betaMax);
            t += dt;
        }
        auto* r = out->as<ob::CompoundState>();
        auto* rse2 = r->as<ob::SE2StateSpace::StateType>(0);
        auto* rvec = r->as<ob::RealVectorStateSpace::StateType>(1);
        rse2->setX(x); rse2->setY(y); rse2->setYaw(psi0);
        rvec->values[0]=d0; rvec->values[1]=beta;
        si_->getStateSpace()->enforceBounds(out);
    }
  private: TrailerParams P_;
};

std::shared_ptr<oc::StatePropagator> makeBicyclePropagator(const oc::SpaceInformationPtr& si,
                                                           BicycleParams P) {
    return std::make_shared<BicycleProp>(si, P);
}
std::shared_ptr<oc::StatePropagator> makeFourWSPropagator(const oc::SpaceInformationPtr& si,
                                                          FourWSParams P) {
    return std::make_shared<FourWSProp>(si, P);
}
std::shared_ptr<oc::StatePropagator> makeTractorTrailerPropagator(const oc::SpaceInformationPtr& si,
                                                                  TrailerParams P) {
    return std::make_shared<TractorTrailerProp>(si, P);
}

// --------------------------- Planner factory ---------------------------
ompl::base::PlannerPtr makePlanner(PlannerType which, const oc::SpaceInformationPtr& si) {
    switch (which) {
        case PlannerType::KPIECE: return std::make_shared<oc::KPIECE1>(si);
        case PlannerType::SST:    return std::make_shared<oc::SST>(si);
    }
    return std::make_shared<oc::KPIECE1>(si);
}

// --------------------------- Setup helpers ---------------------------
SetupBundle createBicycleSetup(const ob::RealVectorBounds& xy, const BicycleParams& P,
                               PlannerType planner, double dt,
                               unsigned minSteps, unsigned maxSteps) {
    SetupBundle B{};
    B.space = buildBicycleStateSpace(xy, P);
    registerDefaultProjections(B.space, ModelType::Bicycle);
    B.control = buildBicycleControlSpace(B.space, P);
    auto si = std::make_shared<oc::SpaceInformation>(B.space, B.control);
    si->setPropagationStepSize(dt);
    si->setMinMaxControlDuration(minSteps, maxSteps);
    B.ss = std::make_shared<oc::SimpleSetup>(si);
    si->setStatePropagator(makeBicyclePropagator(si, P));
    B.ss->setPlanner(makePlanner(planner, si));
    return B;
}

SetupBundle createFourWSSetup(const ob::RealVectorBounds& xy, const FourWSParams& P,
                              PlannerType planner, double dt,
                              unsigned minSteps, unsigned maxSteps) {
    SetupBundle B{};
    B.space = buildFourWSStateSpace(xy, P);
    registerDefaultProjections(B.space, ModelType::FourWS);
    B.control = buildFourWSControlSpace(B.space, P);
    auto si = std::make_shared<oc::SpaceInformation>(B.space, B.control);
    si->setPropagationStepSize(dt);
    si->setMinMaxControlDuration(minSteps, maxSteps);
    B.ss = std::make_shared<oc::SimpleSetup>(si);
    si->setStatePropagator(makeFourWSPropagator(si, P));
    B.ss->setPlanner(makePlanner(planner, si));
    return B;
}

SetupBundle createTractorTrailerSetup(const ob::RealVectorBounds& xy, const TrailerParams& P,
                                      PlannerType planner, double dt,
                                      unsigned minSteps, unsigned maxSteps) {
    SetupBundle B{};
    B.space = buildTractorTrailerStateSpace(xy, P);
    registerDefaultProjections(B.space, ModelType::TractorTrailer);
    B.control = buildTractorTrailerControlSpace(B.space, P);
    auto si = std::make_shared<oc::SpaceInformation>(B.space, B.control);
    si->setPropagationStepSize(dt);
    si->setMinMaxControlDuration(minSteps, maxSteps);
    B.ss = std::make_shared<oc::SimpleSetup>(si);
    si->setStatePropagator(makeTractorTrailerPropagator(si, P));
    B.ss->setPlanner(makePlanner(planner, si));
    return B;
}

} // namespace nhm
