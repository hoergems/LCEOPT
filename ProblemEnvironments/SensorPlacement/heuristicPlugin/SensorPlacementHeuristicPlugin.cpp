#include "oppt/plugin/Plugin.hpp"
#include "SensorPlacementHeuristicOptions.hpp"
#include "SensorPlacementUserData.hpp"
#include "GoalParser.hpp"

namespace oppt
{
class SensorPlacementHeuristicPlugin: public HeuristicPlugin
{
public:
    SensorPlacementHeuristicPlugin():
        HeuristicPlugin() {
    }

    virtual ~SensorPlacementHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementHeuristicOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementHeuristicOptions *>(options_.get());        
        parseGoal_();

        exitReward_ = options->exitReward;
        stepPenalty_ = options->stepPenalty;
        discountFactor_ = options->discountFactor;
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        auto ud = heuristicInfo->currentState->getUserData()->as<SensorPlacementUserData>();
        FloatType distToGoal = (goalPosition_ - ud->eePose.position).norm();
        FloatType estimatedVal = ((1.0 - std::pow(discountFactor_ , distToGoal)) / (1.0 - discountFactor_)) * stepPenalty_;
        estimatedVal += std::pow(discountFactor_, distToGoal - 1.0) * exitReward_;
        return estimatedVal;
    }

private:
    FloatType discountFactor_ = 0.0;

    FloatType stepPenalty_ = 0.0;

    FloatType exitReward_ = 0.0;    

    Vector3f goalPosition_;   

private:
    void parseGoal_() {
        GoalParser p;
        VectorFloat goalArea = p.parseGoalAreaFromFile(robotEnvironment_->getWorldFile());
        goalPosition_ = Vector3f(goalArea[0], goalArea[1], goalArea[2]);
    }

};

OPPT_REGISTER_HEURISTIC_PLUGIN(SensorPlacementHeuristicPlugin)

}