#include "oppt/plugin/Plugin.hpp"
#include "TagState.hpp"
#include "TagObservation.hpp"

namespace oppt
{
class ContTagObservationPlugin: public ObservationPlugin
{
public :
    ContTagObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~ContTagObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        FloatType bearing = getBearing(observationRequest->currentState.get());
        bool visible = false;
        if (bearing < angleUpper_ and bearing > angleLower_) {
            // Possibly visible
            visible = std::bernoulli_distribution(1.0 - bearing / (angleUpper_ - angleLower_))(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
        }
        
        ObservationResultSharedPtr obsRes(new ObservationResult);
        obsRes->observation = ObservationSharedPtr(new TagObservation(visible));
        return obsRes;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const override {        
        bool visible = observation->as<TagObservation>()->visible();
        FloatType bearing = getBearing(state.get());
        FloatType prob = 0.0;        
        if (bearing < angleUpper_ and bearing > angleLower_) {
            FloatType visibleProbability = 1.0 - bearing / (angleUpper_ - angleLower_);
            prob = visible == true ? visibleProbability : 1.0 - visibleProbability;            
        } else {
            prob = visible == false ? 1.0 : 0.0;
        }       

        return prob;
    }

private:
    static constexpr FloatType PiHalf = M_PI / 2.0;
    static constexpr FloatType PiQuarter = M_PI / 4.0;
    static constexpr FloatType TwoPi = 2.0 * M_PI;

    static constexpr FloatType angleLower_ = -PiHalf;
    static constexpr FloatType angleUpper_ = PiHalf;

private:
    FloatType getBearing(const RobotState *state) const {
        return getBearing(state->as<TagState>()->robotPosition(), state->as<TagState>()->targetPosition(), state->as<TagState>()->robotAngle());        
    }

    FloatType getBearing(const Vector2f &robotPosition, const Vector2f &targetPosition, const FloatType &robotAngle) const {
        Position targetDirection = targetPosition - robotPosition;
        FloatType targetDirectionAngle = std::atan2(targetDirection.y(), targetDirection.x());
        return normalizeAngle(targetDirectionAngle - robotAngle);
    } 

    FloatType normalizeAngle(const FloatType &angle) const {
        FloatType normalizedAngle = angle;
        if (normalizedAngle > M_PI) {
            normalizedAngle -= TwoPi;
        } else if (normalizedAngle <= -M_PI) {
            normalizedAngle += TwoPi;
        }

        return normalizedAngle;
    }  

};

OPPT_REGISTER_OBSERVATION_PLUGIN(ContTagObservationPlugin)

}
