#include "oppt/plugin/Plugin.hpp"
#include <oppt/opptCore/geometric/Sphere.hpp>
#include "ParkingHeuristicOptions.hpp"
#include "VehicleState.hpp"

namespace oppt
{
class ParkingHeuristicPlugin: public HeuristicPlugin
{
public:
    ParkingHeuristicPlugin():
        HeuristicPlugin() {
    }

    virtual ~ParkingHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ParkingHeuristicOptions>(optionsFile);
        parseGoalArea();
        auto options = static_cast<const ParkingHeuristicOptions *>(options_.get());
        stepPenalty = options->stepPenalty;
        exitReward = options->exitReward;
        return true;
    }

    virtual double getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        auto options = static_cast<const ParkingHeuristicOptions *>(options_.get());
        auto vehicleState = heuristicInfo->currentState->as<VehicleState>();
        VectorFloat vehiclePos({vehicleState->x(), vehicleState->y(), vehicleState->z()});
        FloatType distanceToGoal = math::euclideanDistance(vehiclePos, goalAreaPosition_);
        FloatType estimatedVal = ((1.0 - std::pow(options->discountFactor, distanceToGoal)) / (1.0 - options->discountFactor)) * stepPenalty;
        estimatedVal += std::pow(options->discountFactor, distanceToGoal - 1.0) * exitReward;
        return estimatedVal;
    }

private:
    VectorFloat goalAreaPosition_;

    FloatType goalAreaRadius_ = 1.0;

    FloatType stepPenalty = 0.0;

    FloatType exitReward = 0.0;

private:
    void parseGoalArea() {
        auto bodies = robotEnvironment_->getScene()->getBodies();
        for (auto &body: bodies) {
            if (body->getName() == "GoalArea") {
                auto pose = body->getWorldPose();                
                goalAreaPosition_ = VectorFloat({pose.position.x(), pose.position.y(), pose.position.z()});
                goalAreaRadius_ = body->getVisualGeometries()[0]->as<geometric::Sphere>()->getRadius();
                
            }            
        }        
    }
};

OPPT_REGISTER_HEURISTIC_PLUGIN(ParkingHeuristicPlugin)

}