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
#include "Pushbox3DState.hpp"
#include <oppt/opptCore/Distribution.hpp>

namespace oppt
{
class Pushbox3DObservationPlugin: public ObservationPlugin
{
public :
    Pushbox3DObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~Pushbox3DObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxObservationPluginOptions>(optionsFile);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        observationUncertainty_ = options->observationUncertainty * M_PI / 180.0;
        observationDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, observationUncertainty_);
        observationBucketFactor_ = (360.0 / ((FloatType)(options->numberOfObservationBuckets))) * M_PI / 180.0;
        positionObservation = options->usePositionObservation;

        for (size_t i = 0; i != options->numberOfObservationBuckets + 1; ++i) {
            bearingBoundaries_.emplace_back(i * observationBucketFactor_);

        }

        if (positionObservation) {
            probs = VectorFloat({0.1, 0.8, 0.1});
            distDistr_ = std::make_unique<std::discrete_distribution<int>>(probs.begin(), probs.end());
        }

        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        if (positionObservation)
            return getPositionObservation(observationRequest);
        return getBearingObservation(observationRequest);
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const override {
        if (positionObservation)
            return calcLikelihoodPosition(state, action, observation);
        return calcLikelihoodBearing(state, action, observation);        
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> observationDistribution_ = nullptr;

    static constexpr FloatType TwoPi = 2.0 * M_PI;

    FloatType observationUncertainty_ = 0.0;

    FloatType observationBucketFactor_ = 0.0;

    VectorFloat bearingBoundaries_;

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<bool, int>>> binMap_;

    bool positionObservation = false;

    VectorFloat probs;

    std::unique_ptr<std::discrete_distribution<int>> distDistr_ = nullptr;

    ObservationResultSharedPtr getBearingObservation(const ObservationRequest* observationRequest) const {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        auto options = static_cast<const PushboxObservationPluginOptions *>(options_.get());
        auto state = observationRequest->currentState->as<Pushbox3DState>();

        // Compute the angle between the robot and the target
        Vector3f delta = state->targetPosition() - state->robotPosition();
        FloatType angle1 = std::atan2(delta.y(), delta.x());
        FloatType angle2 = std::atan2(delta.z(), delta.y());

        // Add observation noise to the angle1 and angle2
        angle1 += observationDistribution_->sample(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
        angle2 += observationDistribution_->sample(*(robotEnvironment_->getRobot()->getRandomEngine().get()));

        // Normalize the angle1 and angle2 to [0, 2Pi]
        if (angle1 < 0) angle1 += TwoPi;
        if (angle1 > TwoPi) angle1 -= TwoPi;
        if (angle1 < 0) angle1 = 0.0;

        if (angle2 < 0) angle2 += TwoPi;
        if (angle2 > TwoPi) angle2 -= TwoPi;
        if (angle2 < 0) angle2 = 0.0;

        // Get the bearing observation from the angle1 and angle2
        int bearing1 = std::floor(angle1 / observationBucketFactor_);
        int bearing2 = std::floor(angle2 / observationBucketFactor_);

        bool pushed = observationRequest->currentState->getUserData()->as<PushboxStateUserData>()->pushed;
        int p = pushed ? 1 : 0;
        int binNumber =
            p * options->numberOfObservationBuckets * options->numberOfObservationBuckets + (bearing1 * options->numberOfObservationBuckets + bearing2);

        // Observed the push
        observationResult->observation = ObservationSharedPtr(new BearingObservation(VectorInt({bearing1, bearing2}), pushed));
        observationResult->observation->as<DiscreteVectorObservation>()->setBinNumber(binNumber);

        return observationResult;
    }

    FloatType calcLikelihoodBearing(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const {
        auto pushboxState = state->as<Pushbox3DState>();
        auto obs = observation->as<BearingObservation>();

        // If push of the state is different to the observed push => zero probability
        if (pushboxState->getUserData()->as<PushboxStateUserData>()->pushed != obs->getPushed()) {
            return 0.0;
        }

        Vector3f delta = pushboxState->targetPosition() - pushboxState->robotPosition();
        FloatType prob = 1.0;
        for (size_t k = 0; k != 2; ++k) {
            // Get angle between robot and target corresponding to state
            FloatType angle = 0.0;
            if (k == 0) {
                angle = std::atan2(delta.y(), delta.x());
            } else {
                angle = std::atan2(delta.z(), delta.y());
            }

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
                FloatType ub = bearingBoundariesCentered[i + 1];
                while (lb > ub) {
                    ub += TwoPi;
                }

                // Compute CDF of lower bearing boundary minus CDF of upper bearing boundary, which gives
                // the probability of observing bearing 'i', given the angle1 corresponding to the state
                bearingProbs.emplace_back(observationDistribution_->cdf(ub) - observationDistribution_->cdf(lb));
            }

            prob *= bearingProbs[obs->getBearings()[k]];
            //cout << "prob: " << prob << endl;
        }

        return prob;

    }

    FloatType calcLikelihoodPosition(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const {
        auto pushboxState = state->as<Pushbox3DState>();
        auto obs = observation->as<DistObservation>();

        if (pushboxState->getUserData()->as<PushboxStateUserData>()->pushed != obs->pushed()) {
            return 0.0;
        }

        int dist = std::floor((pushboxState->targetPosition() - pushboxState->robotPosition()).norm());
        int obsInt = observation->as<DiscreteVectorObservation>()->getBinNumber();
        FloatType prob = 0.0;
        if (dist == 0) {
            if (obsInt == 0) {
                prob = 0.9;
            } else if (obsInt == 1) {
                prob = 0.1;
            } 
        } else if (dist == 10) {
            if (obsInt == 10) {
                prob = 0.9;
            } else if (obsInt == 9) {
                prob = 0.1;
            }
        } else {
            if (obsInt == dist - 1 or obsInt == dist + 1) {
                prob = 0.1;
            } else if (obsInt == dist) {
                prob = 0.8;
            }
        }

        return prob;
    }

    ObservationResultSharedPtr getPositionObservation(const ObservationRequest* observationRequest) const {
        auto state = observationRequest->currentState->as<Pushbox3DState>();
        Vector3f delta = state->targetPosition() - state->robotPosition();
        FloatType dist = delta.norm();

        int obs = std::floor(dist);

        int idx = distDistr_->operator()(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
        if (idx == 0) {
            obs -= 1;
            if (obs < 0)
                obs = 0;
        } else if (idx == 2) {
            obs += 1;
            if (obs > 10)
                obs = 10;
        }           

        ObservationResultSharedPtr obsRes(new ObservationResult);
        obsRes->observation = ObservationSharedPtr(new DistObservation(obs, state->getUserData()->as<PushboxStateUserData>()->pushed));        
        return obsRes;
    }
};

OPPT_REGISTER_OBSERVATION_PLUGIN(Pushbox3DObservationPlugin)

}
