/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#include "oppt/plugin/Plugin.hpp"
#include "PushboxObservationPluginOptions.hpp"
#include "TruncatedNormal.hpp"
#include "PushboxStateUserData.hpp"
#include "BearingObservation.hpp"
#include "Pushbox2DState.hpp"
#include <oppt/opptCore/Distribution.hpp>

namespace oppt
{
class Pushbox2DObservationPlugin: public ObservationPlugin
{
public :
    Pushbox2DObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~Pushbox2DObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxObservationPluginOptions>(optionsFile);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        observationUncertainty_ = options->observationUncertainty * M_PI / 180.0;
        observationDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, observationUncertainty_);        
        observationBucketFactor_ = (360.0 / ((FloatType)(options->numberOfObservationBuckets))) * M_PI / 180.0;
        for (size_t i = 0; i != options->numberOfObservationBuckets+1; ++i) {
            bearingBoundaries_.emplace_back(i * observationBucketFactor_);
        }
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {        
        ObservationResultSharedPtr observationResult(new ObservationResult);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        auto state = observationRequest->currentState->as<Pushbox2DState>();

        // Compute the angle between the robot and the target
        Vector2f delta = state->targetPosition() - state->robotPosition();
        FloatType angle = std::atan2(delta.y(), delta.x());

        // Add observation noise to the angle
        angle += observationDistribution_->sample(*(robotEnvironment_->getRobot()->getRandomEngine().get()));

        // Normalize the angle to [0, 2Pi]
        if (angle < 0) angle += TwoPi;
        if (angle > TwoPi) angle -= TwoPi;
        if (angle < 0) angle = 0.0;

        // Get the bearing observation from the angle
        int bearing = std::floor(angle / observationBucketFactor_);

        // Observed the push       
        bool pushed = observationRequest->currentState->getUserData()->as<PushboxStateUserData>()->pushed;
        observationResult->observation = ObservationSharedPtr(new BearingObservation(VectorInt({bearing}), pushed));
        if (pushed) {
            observationResult->observation->as<DiscreteVectorObservation>()->setBinNumber(bearing + options->numberOfObservationBuckets);
        } else {            
            observationResult->observation->as<DiscreteVectorObservation>()->setBinNumber(bearing);
        }
        return observationResult;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const override {
        auto pushboxState = state->as<Pushbox2DState>();
        auto obs = observation->as<BearingObservation>();

        // If push of the state is different to the observed push => zero probability
        if (pushboxState->getUserData()->as<PushboxStateUserData>()->pushed != obs->getPushed())
            return 0.0;
        
        Vector2f delta = pushboxState->targetPosition() - pushboxState->robotPosition();

        // Get angle between robot and target corresponding to state
        FloatType angle = std::atan2(delta.y(), delta.x());
        if (angle < 0) angle += TwoPi;
        if (angle > TwoPi) angle -= TwoPi;
        if (angle < 0) angle = 0.0;        

        // Express the bearing boundaries with respect to the angle
        VectorFloat bearingBoundariesCentered(bearingBoundaries_.size(), 0.0);
        for (size_t i = 0; i != bearingBoundariesCentered.size(); ++i) {
            bearingBoundariesCentered[i] = math::distAngles(bearingBoundaries_[i], angle);            
        }

        // Calc probabilies for each bearing, given the angle corresponding to state
        VectorFloat bearingProbs;
        for (size_t i = 0; i != bearingBoundariesCentered.size() - 1; ++i) {
            FloatType lb = bearingBoundariesCentered[i];
            FloatType ub = bearingBoundariesCentered[i+1];            
            while (lb > ub) {
                ub += TwoPi;
            }

            // Compute CDF of lower bearing boundary minus CDF of upper bearing boundary, which gives 
            // the probability of observing bearing 'i', given the angle corresponding to the state                       
            bearingProbs.emplace_back(observationDistribution_->cdf(ub) - observationDistribution_->cdf(lb));
        }

        // Return probability of observed bearing
        return bearingProbs[obs->getBearings()[0]];
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> observationDistribution_ = nullptr;

    static constexpr FloatType TwoPi = 2.0 * M_PI;

    FloatType observationUncertainty_ = 0.0;

    FloatType observationBucketFactor_ = 0.0; 

    VectorFloat bearingBoundaries_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(Pushbox2DObservationPlugin)

}
