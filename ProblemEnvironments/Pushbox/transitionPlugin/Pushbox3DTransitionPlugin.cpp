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
#include "TruncatedNormal.hpp"
#include "PushboxTransitionPluginOptions.hpp"
#include "PushboxStateUserData.hpp"
#include "Map2d.hpp"
#include "Pushbox3DState.hpp"
#include "MapString.hpp"
#ifdef GZ_SUPPORT
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#endif

namespace oppt
{

// Transition plugin for multi-dimensional pushbox problem
class PushboxTransitionPlugin: public TransitionPlugin
{
public :
    PushboxTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~PushboxTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<PushboxTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const PushboxTransitionPluginOptions *>(options_.get());
        boxSpeedFactorDistribution_ = std::make_unique<TruncatedNormalDistribution>(1.0, options->boxSpeedUncertainty);
        boxAbsoluteDistribution_ = std::make_unique<TruncatedNormalDistribution>(0.0, options->boxPositionMoveUncertainty);

        // Map
        map_ = Map(options->sizeX, options->sizeY, ' ');

        std::istringstream is(mapString);        
        map_.loadFromStream(is);
        //map_.loadFromFile(options->mapPath);

#ifdef GZ_SUPPORT
        if (options->robotName != "Default") {
            robotLink_ = getLinkPtr(robotEnvironment_, "PushboxRobot::PushboxRobotLink");
            opponentLink_ = getLinkPtr(robotEnvironment_, "PushboxOpponent::PushboxOpponentLink");
            if (robotLink_ == nullptr)
                ERROR("Robot link couldn't be found");
            if (opponentLink_ == nullptr)
                ERROR("Opponent link couldn't be found");
            updateGazebo = true;
        }
#endif

        goalPosition_ = Vector3f(options->goalPosition[0], options->goalPosition[1], options->goalPosition[2]);
        goalRadius_ = options->goalRadius;

        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        auto options = static_cast<const PushboxTransitionPluginOptions *>(options_.get());
        PropagationResultSharedPtr propRes(new PropagationResult);
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        auto state = propagationRequest->currentState->as<Pushbox3DState>();
        Position robotPosition = state->robotPosition();
        Position boxPosition = state->targetPosition();
        Position nextBoxPosition(boxPosition);
        VectorFloat delta = propagationRequest->action->as<VectorAction>()->asVector();       
        FloatType speedSquared = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
        bool pushed = false;
        if (speedSquared > 0) {
            FloatType speed = std::sqrt(speedSquared);

            // first get the vector from the robot position to the box position.
            Vector3f toBox = boxPosition - robotPosition;

            // then project it onto the line defined by deltaX, deltaY
            FloatType t = 0.0;
            t += toBox.x() * delta[0];
            t += toBox.y() * delta[1];
            t += toBox.z() * delta[2];
            t /= speedSquared;


            Vector3f closest(robotPosition.x() + delta[0]*t, robotPosition.y() + delta[1]*t, robotPosition.z() + delta[2]*t);
            FloatType lineToBoxDistanceSquared = 
            std::pow(boxPosition.x() - closest.x(), 2) + std::pow(boxPosition.y() - closest.y(), 2) + std::pow(boxPosition.z() - closest.z(), 2);

            // only proceed if we have a proper contact.
            if (lineToBoxDistanceSquared < 1) {
                // how far is the intersection from the 'closest point'.
                FloatType intersectionDistance = std::sqrt(1 - lineToBoxDistanceSquared);

                // we are only interested in the closer solution there would be a second one with a +.
                FloatType contactT = t - intersectionDistance / speed;
                if ( (0 <= contactT) && (contactT <= 1 ) ) {
                    // we have a collision.
                    Vector3f contact(robotPosition.x() + delta[0] * contactT, robotPosition.y() + delta[1] * contactT, robotPosition.z() + delta[2] * contactT);

                    // get the vector describing the collision direction. It is unit length by definition of there being contact.
                    Vector3f boxDirection = boxPosition - contact;

                    // the box speed is dependent on how fast we bump and the direction we bump (scalar product).
                    FloatType boxSpeed = delta[0] * boxDirection.x() + delta[1] * boxDirection.y() + delta[2] * boxDirection.z();

                    // make the box move a bit faster
                    boxSpeed *= 5;
                    // ...and random..
                    boxSpeed *= boxSpeedFactorDistribution_->sample(*randomEngine);

                    // make a new box position
                    nextBoxPosition.x() += boxDirection.x() * boxSpeed + boxSpeed * boxAbsoluteDistribution_->sample(*randomEngine);
                    nextBoxPosition.y() += boxDirection.y() * boxSpeed + boxSpeed * boxAbsoluteDistribution_->sample(*randomEngine);
                    nextBoxPosition.z() += boxDirection.z() * boxSpeed + boxSpeed * boxAbsoluteDistribution_->sample(*randomEngine);

                    pushed = true;
                }
            }            
        }

        Vector3f nextRobotPosition(robotPosition.x() + delta[0], robotPosition.y() + delta[1], robotPosition.z() + delta[2]);

        propRes->nextState = RobotStateSharedPtr(new Pushbox3DState(nextRobotPosition, nextBoxPosition));
        OpptUserDataSharedPtr userData(new PushboxStateUserData);
        userData->as<PushboxStateUserData>()->previousState = propagationRequest->currentState;
        userData->as<PushboxStateUserData>()->isGoalState = isGoalState(nextBoxPosition);
        userData->as<PushboxStateUserData>()->isInCollision = inCollision(nextRobotPosition, nextBoxPosition);
        userData->as<PushboxStateUserData>()->pushed = pushed;
        propRes->nextState->setUserData(userData);

#ifdef GZ_SUPPORT
        if (updateGazebo and (robotEnvironment_->isExecutionEnvironment() or
                              (options->particlePlotLimit > 0 and propagationRequest->userData != nullptr))) {
            geometric::Pose robotPose(nextRobotPosition.x(), nextRobotPosition.y(), nextRobotPosition.z(), 0.0, 0.0, 0.0);
            geometric::Pose opponentPose(nextBoxPosition.x(), nextBoxPosition.y(), nextBoxPosition.z(), 0.0, 0.0, 0.0);
            robotLink_->SetWorldPose(robotPose.toGZPose());
            opponentLink_->SetWorldPose(opponentPose.toGZPose());


            propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        }
#endif


        return propRes;
    }

private:
    std::unique_ptr<TruncatedNormalDistribution> boxSpeedFactorDistribution_ = nullptr;

    std::unique_ptr<TruncatedNormalDistribution> boxAbsoluteDistribution_ = nullptr;

    Map map_;

#ifdef GZ_SUPPORT
    gazebo::physics::Link* robotLink_ = nullptr;

    gazebo::physics::Link* opponentLink_ = nullptr;
#endif

    bool updateGazebo = false;

    Vector3f goalPosition_;

    FloatType goalRadius_ = 0.0;

private:
    bool isGoalState(const Position &boxPosition) const {
        return (goalPosition_ - boxPosition).norm() <= goalRadius_;
        //return lookupInMap(boxPosition.x(), boxPosition.y()) == 'g';
    }

    bool inCollision(const Position &robotPosition, const Position &boxPosition) const {
        if (lookupInMap(robotPosition.x(), robotPosition.y()) == '#') return true;
        if (lookupInMap(boxPosition.x(), boxPosition.y()) == '#') return true;
        if ((robotPosition - boxPosition).squaredNorm() < 1) return true;
        return false;
    }

    char lookupInMap(const FloatType &x, const FloatType &y) const {
        if ( (x < 0) || (y < 0) ) return '#';
        size_t x_ = std::floor(x);
        size_t y_ = std::floor(y);
        if ( (x_ >= map_.sizeX()) || (y_ >= map_.sizeY()) ) return '#';
        return map_(x_, y_);
    }


#ifdef GZ_SUPPORT
    gazebo::physics::Link *getLinkPtr(const RobotEnvironment *robotEnvironment, const std::string &name) {
        auto links = robotEnvironment->getGazeboInterface()->getLinks();
        gazebo::physics::Link *foundLink = nullptr;
        for (auto &link : links) {
            if (link->GetScopedName() == name) {
                foundLink = link;
                break;
            }
        }

        if (!foundLink) {
            WARNING("Link '" + name + "' could not be found");
            getchar();
        }

        return foundLink;
    }
#endif
};

OPPT_REGISTER_TRANSITION_PLUGIN(PushboxTransitionPlugin)

}
