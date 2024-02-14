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
#include "PushboxRewardOptions.hpp"
#include "PushboxStateUserData.hpp"

namespace oppt
{
class PushboxRewardPlugin: public RewardPlugin
{
public :
    PushboxRewardPlugin():
        RewardPlugin() {

    }

    virtual ~PushboxRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxRewardOptions>(optionsFile);
        auto options = static_cast<const PushboxRewardOptions *>(options_.get());
        moveCost_ = options->moveCost;
        goalReward_ = options->goalReward;
        collisionPenalty_ = options->collisionPenalty;
        return true;       
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        auto userData = propagationResult->nextState->getUserData()->as<PushboxStateUserData>();
        FloatType reward = -moveCost_;
        if (userData->isGoalState)
            reward += goalReward_;
        if (userData->isInCollision)
            reward -= collisionPenalty_;
        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::pair<FloatType, FloatType>(-moveCost_ - collisionPenalty_, -moveCost_ + goalReward_);
    }

private:
    FloatType moveCost_ = 0.0;
    FloatType goalReward_ = 0.0;
    FloatType collisionPenalty_ = 0.0;
};

OPPT_REGISTER_REWARD_PLUGIN(PushboxRewardPlugin)

}
