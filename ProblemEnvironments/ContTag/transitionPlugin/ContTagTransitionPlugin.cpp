#include "oppt/plugin/Plugin.hpp"
#include "ContTagTransitionPluginOptions.hpp"
#include "TagState.hpp"
#include "TagUserData.hpp"
#include "TruncatedNormal.hpp"
#ifdef GZ_SUPPORT
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#endif

namespace oppt
{

class ContTagTransitionPlugin: public TransitionPlugin
{
public :
    ContTagTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~ContTagTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ContTagTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const ContTagTransitionPluginOptions *>(options_.get());

        std::istringstream is("#####...##\n#####...##\n#####...##\n..........\n..........");
        map_.loadFromStream(is);
        //map_.loadFromFile(options->mapPath);
        targetAngleDistribution_ = TruncatedNormalPtr(new TruncatedNormalDistribution(0, targetAngleUncertainty_));

#ifdef GZ_SUPPORT
        agentLink_ = getLinkPtr(robotEnvironment_, "Agent::AgentLink");
        targetLink_ = getLinkPtr(robotEnvironment_, "Target::TargetLink");
#endif
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        auto action = propagationRequest->action->as<VectorAction>()->asVector();
        auto currentState = propagationRequest->currentState->as<TagState>();
        auto newRobotPosition = currentState->robotPosition();
        auto newTargetPosition = currentState->targetPosition();
        auto newRobotangle = currentState->robotAngle();
        bool isTagged = currentState->tagged();

        bool movedIntoWall = false;

        Position targetDirection = newTargetPosition - newRobotPosition;
        bool tagged = false;
        if (action[1] > 0.5) {
            // Tag action
            if (targetDirection.norm() <= tagRange_)
                tagged = true;
        } else {            
            newRobotangle += 2.0*M_PI * action[0]-M_PI;
            if (robotEnvironment_->isExecutionEnvironment())
                LOGGING("RES ROBOT ANGLE: " + std::to_string(newRobotangle));
            newRobotPosition.x() += cos(newRobotangle) * moveDistance_;
            newRobotPosition.y() += sin(newRobotangle) * moveDistance_;

            if (lookupInMap(newRobotPosition) == '#') {
                newRobotangle = currentState->robotAngle();
                newRobotPosition = currentState->robotPosition();
                movedIntoWall = true;
            }
        }

        newRobotangle = normalizeAngle(newRobotangle);

        if (tagged == false) {
            // Make new target positions
            FloatType targetDirectionAngle;
            if ( (targetDirection.x() == 0) && (targetDirection.y() == 0) ) {
                // just to make sure we don't crash...
                targetDirectionAngle = currentState->robotAngle();
            } else {
                targetDirectionAngle = std::atan2(targetDirection.y(), targetDirection.x());
            }

            targetDirectionAngle += targetAngleDistribution_->sample(*randomEngine);
            if (robotEnvironment_->isExecutionEnvironment())
                cout << "targetDirectionAngle: " << targetDirectionAngle << endl;

            newTargetPosition.x() += cos(targetDirectionAngle) * targetMoveDistance_;
            newTargetPosition.y() += sin(targetDirectionAngle) * targetMoveDistance_;

            if (lookupInMap(newTargetPosition) == '#') {
                newTargetPosition = currentState->targetPosition();
            }
        }

        PropagationResultSharedPtr propRes(new PropagationResult);
        propRes->nextState = RobotStateSharedPtr(new TagState(newRobotPosition, newRobotangle, newTargetPosition, tagged));

        OpptUserDataSharedPtr ud(new TagUserData);
        ud->as<TagUserData>()->movedIntoWall = movedIntoWall;
        propRes->nextState->setUserData(ud);

        // Visualization
#ifdef GZ_SUPPORT
        if (robotEnvironment_->isExecutionEnvironment() or
                (static_cast<const ContTagTransitionPluginOptions *>(options_.get())->particlePlotLimit > 0 and
                 propagationRequest->userData != nullptr)) {
            geometric::Pose agentPose(newRobotPosition.x(),
                                      newRobotPosition.y(),
                                      0.0,
                                      0.0,
                                      0.0,
                                      newRobotangle);
            geometric::Pose targetPose(newTargetPosition.x(),
                                       newTargetPosition.y(),
                                       0.0,
                                       0.0,
                                       0.0,
                                       0.0);
            agentLink_->SetWorldPose(agentPose.toGZPose());
            targetLink_->SetWorldPose(targetPose.toGZPose());
            propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        }
#endif

        return propRes;
    }

private:
    static constexpr float tagRange_ = 1.0;

    static constexpr FloatType moveDistance_ = 1.0;

    static constexpr FloatType targetMoveDistance_ = 1.0;

    static constexpr FloatType targetAngleUncertainty_ = M_PI / 8.0;

    static constexpr FloatType TwoPi = 2.0 * M_PI;

    TruncatedNormalPtr targetAngleDistribution_ = nullptr;

#ifdef GZ_SUPPORT
    gazebo::physics::Link* agentLink_ = nullptr;

    gazebo::physics::Link* targetLink_ = nullptr;
#endif

    Map map_;

private:
    FloatType normalizeAngle(const FloatType &angle) const {
        FloatType normalizedAngle = angle;
        if (normalizedAngle > M_PI) {
            normalizedAngle -= TwoPi;
        } else if (normalizedAngle <= -M_PI) {
            normalizedAngle += TwoPi;
        }

        return normalizedAngle;
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

    char lookupInMap(const Position &position) const {
        if ( (position.x() < 0) || (position.y() < 0) ) return '#';
        size_t int_x = std::floor(position.x());
        size_t int_y = std::floor(position.y());
        if ( (int_x >= map_.sizeX()) || (int_y >= map_.sizeY()) ) return '#';
        return map_(int_x, int_y);
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(ContTagTransitionPlugin)

}