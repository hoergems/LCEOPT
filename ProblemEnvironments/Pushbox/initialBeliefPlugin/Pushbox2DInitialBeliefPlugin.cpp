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
#include "PushboxInitialBeliefOptions.hpp"
#include "TruncatedNormal.hpp"
#include "PushboxStateUserData.hpp"
#include "Pushbox2DState.hpp"

namespace oppt
{



class Pushbox2DInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    Pushbox2DInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~Pushbox2DInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxInitialBeliefOptions>(optionsFile);
        auto options = static_cast<const PushboxInitialBeliefOptions *>(options_.get());
        truncatedNormal_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->initialBoxPositionUncertainty);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        VectorFloat initialStateVec = static_cast<const PushboxInitialBeliefOptions *>(options_.get())->initialStateVec;
        Position initRobotPosition;
        Position initBoxPosition;
        initRobotPosition.x() = initialStateVec[0];
        initRobotPosition.y() = initialStateVec[1];

        initBoxPosition.x() = initialStateVec[2] + truncatedNormal_->sample((*(robotEnvironment_->getRobot()->getRandomEngine().get())));
        initBoxPosition.y() = initialStateVec[3] + truncatedNormal_->sample((*(robotEnvironment_->getRobot()->getRandomEngine().get())));
        

        RobotStateSharedPtr initialState(new Pushbox2DState(initRobotPosition, initBoxPosition));
        OpptUserDataSharedPtr userData(new PushboxStateUserData);
        userData->as<PushboxStateUserData>()->isGoalState = false;
        userData->as<PushboxStateUserData>()->isInCollision = false;
        initialState->setUserData(userData);
        return initialState;
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> truncatedNormal_ = nullptr;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(Pushbox2DInitialBeliefPlugin)

}