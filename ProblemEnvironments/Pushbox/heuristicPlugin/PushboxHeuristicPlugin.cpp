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
#include "PushboxStateUserData.hpp"
#include "PushboxHeuristicOptions.hpp"

namespace oppt
{
class PushboxHeuristicPlugin: public HeuristicPlugin
{
public:
    PushboxHeuristicPlugin():
        HeuristicPlugin() {

    }

    virtual ~PushboxHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxHeuristicOptions>(optionsFile);
        auto options = static_cast<const PushboxHeuristicOptions *>(options_.get());
        moveCost_ = options->moveCost;
        goalReward_ = options->goalReward;
        goalPosition_ = options->goalPosition;

        // Extend the goal position dimension
        unsigned int numStateDimensionsHalf = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions() / 2;
        if (numStateDimensionsHalf > 2) {
            VectorFloat numAdditionalGoalPositionDimensions(numStateDimensionsHalf - 2, 0.0);
            goalPosition_.insert(goalPosition_.end(), numAdditionalGoalPositionDimensions.begin(), numAdditionalGoalPositionDimensions.end());
        }
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        auto options = static_cast<const PushboxHeuristicOptions *>(options_.get());
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        unsigned int stateHalfDim = stateVec.size() / 2;
        VectorFloat robotPosition(stateVec.begin(), stateVec.begin() + stateHalfDim);
        VectorFloat opponentPosition(stateVec.begin() + stateHalfDim, stateVec.end());
        FloatType distToGoal = math::euclideanDistance(opponentPosition, goalPosition_);
        VectorFloat goalToOpponent = subtractVectors(opponentPosition, goalPosition_);
        opponentPosition = addVectors(opponentPosition, scaleVector(normalizeVector(goalToOpponent), 0.5));
        FloatType dist = math::euclideanDistance(robotPosition, opponentPosition) + math::euclideanDistance(opponentPosition, goalPosition_);
        FloatType result = ((1.0 - std::pow(options->discountFactor, dist + 1.0)) / (1.0 - options->discountFactor)) * moveCost_;
        result += std::pow(options->discountFactor, dist) * goalReward_;
        return result;
    }

private:
    FloatType moveCost_ = 0.0;
    FloatType goalReward_ = 0.0;
    VectorFloat goalPosition_;
};

OPPT_REGISTER_HEURISTIC_PLUGIN(PushboxHeuristicPlugin)

}