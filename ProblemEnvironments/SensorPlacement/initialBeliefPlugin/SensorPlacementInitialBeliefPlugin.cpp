#include "oppt/plugin/Plugin.hpp"
#include "SensorPlacementInitialBeliefOptions.hpp"
#include "SensorPlacementUserData.hpp"

namespace oppt
{
class SensorPlacementInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    SensorPlacementInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~SensorPlacementInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementInitialBeliefOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementInitialBeliefOptions *>(options_.get());
        auto re = robotEnvironment_->getRobot()->getRandomEngine();
        VectorFloat lowerBound = options->lowerUpperBound;
        VectorFloat upperBound = options->lowerUpperBound;
        for (size_t i = 0; i != lowerBound.size(); ++i) {
            lowerBound[i] *= -1.0;
        }

        VectorFloat l({0.0});
        VectorFloat u({1.0});

        distr_ = std::make_unique<UniformDistribution<FloatType>>(lowerBound, upperBound, re);
        distr2_ = std::make_unique<UniformDistribution<FloatType>>(l, u, re);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        VectorFloat initialStateVec = static_cast<const SensorPlacementInitialBeliefOptions *>(options_.get())->initialStateVec;
        FloatType firstAngle = initialStateVec[0];
        initialStateVec = addVectors(initialStateVec, toStdVec<FloatType>(distr_->sample(1).col(0)));
        RobotStateSharedPtr initialState(new VectorState(initialStateVec));
        OpptUserDataSharedPtr userData(new SensorPlacementUserData);
        userData->as<SensorPlacementUserData>()->collides = false;
        userData->as<SensorPlacementUserData>()->eeTouchesWall = false;
        userData->as<SensorPlacementUserData>()->reachedGoal = false;
        initialState->setUserData(userData);
        return initialState;
    }

private:
    std::unique_ptr<UniformDistribution<FloatType>> distr_ = nullptr;

    std::unique_ptr<UniformDistribution<FloatType>> distr2_ = nullptr;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(SensorPlacementInitialBeliefPlugin)

}