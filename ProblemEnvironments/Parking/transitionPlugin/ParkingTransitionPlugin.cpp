#include "oppt/plugin/Plugin.hpp"
#include <oppt/opptCore/geometric/Sphere.hpp>
#include <oppt/opptCore/geometric/Box.hpp>
#include <oppt/opptCore/CollisionRequest.hpp>
#include "ParkingTransitionPluginOptions.hpp"
#include "VehicleState.hpp"
#include "VehicleUserData.hpp"
#ifdef GZ_SUPPORT
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#endif

namespace oppt
{
class Tile {
public:
    FloatType minX = 0.0;
    FloatType maxX = 0.0;

    FloatType minY = 0.0;
    FloatType maxY = 0.0;

    FloatType minZ = 0.0;
    FloatType maxZ = 0.0;

    bool insideTile(const float &x, const float &y) const {
        if (x <= maxX and x >= minX and
                y <= maxY and y >= minY) {
            return true;
        }

        return false;
    }
};


class ParkingTransitionPlugin: public TransitionPlugin
{
public :
    ParkingTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~ParkingTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<ParkingTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const ParkingTransitionPluginOptions *>(options_.get());
        controlDuration_ = options->controlDuration;
        numIntegrationSteps = options->numIntegrationSteps;
        numStateDims_ = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions();

        controlDurationStep_ = controlDuration_ / ((FloatType)(numIntegrationSteps));

#ifdef GZ_SUPPORT
        cout << "ROBOT NAME: " << options->robotName << endl;
        if (options->robotName != "Default") {
            vehicleLink_ = getLinkPtr(robotEnvironment_, "Vehicle::link");
            if (vehicleLink_ == nullptr)
                ERROR("Vehicle::link could not be found");
            updateGazebo = true;
        }
#endif
        parseGoalArea();
        makeErrorDistributions();
        makeTiles();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        auto options = static_cast<const ParkingTransitionPluginOptions *>(options_.get());
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine().get();
        VehicleState *currentState = propagationRequest->currentState->as<VehicleState>();
        RobotStateSharedPtr nextState(new VehicleState(currentState->asVector()));
        auto nextVehicleState = nextState->as<VehicleState>();
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();
        actionVec[0] += (*(velocityErrorDistribution_.get()))(*randomEngine);
        actionVec[1] += (*(yawErrorDistribution_.get()))(*randomEngine);
        if (actionVec.size() != 2)
            actionVec[2] += (*(elevationErrorDistribution_.get()))(*randomEngine);

        bool collides = false;
        geometric::Pose nextVehiclePose;
        for (size_t i = 0; i != numIntegrationSteps; ++i) {
            nextVehicleState->x() = nextVehicleState->x() + controlDurationStep_ * nextVehicleState->velocity() * cos(nextVehicleState->yaw());
            nextVehicleState->y() = nextVehicleState->y() + controlDurationStep_ * nextVehicleState->velocity() * sin(nextVehicleState->yaw());
            if (actionVec.size() != 2) {
                nextVehicleState->z() = nextVehicleState->z() + controlDurationStep_ * actionVec[2];
                if (nextVehicleState->z() > 5.0) {
                    nextVehicleState->z() = 5.0;
                } else if (nextVehicleState->z() < -5.0) {
                    nextVehicleState->z() = -5.0;
                }
            }

            nextVehicleState->yaw() = math::wrapAngle(nextVehicleState->yaw() + controlDurationStep_ * actionVec[1]);
            nextVehicleState->velocity() = nextVehicleState->velocity() + controlDurationStep_ * actionVec[0];
            nextVehiclePose = geometric::Pose(nextVehicleState->x(),
                                              nextVehicleState->y(),
                                              nextVehicleState->z(),
                                              0.0,
                                              0.0,
                                              nextVehicleState->yaw());
            if (collides_(nextVehiclePose)) {
                collides = true;
                break;
            }
        }


        PropagationResultSharedPtr propRes(new PropagationResult);
        propRes->nextState = nextState;
        propRes->nextState->setUserData(makeUserData(nextVehicleState, actionVec, collides));

#ifdef GZ_SUPPORT
        if (updateGazebo == true and (robotEnvironment_->isExecutionEnvironment() or (options->particlePlotLimit > 0 and propagationRequest->userData != nullptr))) {
            vehicleLink_->SetWorldPose(nextVehiclePose.toGZPose());
            propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        }
#endif

        return propRes;
    }

private:
    unsigned int numStateDims_ = 0;

    FloatType controlDuration_ = 0.0;

    FloatType controlDurationStep_ = 0.0;

    unsigned int numIntegrationSteps = 0;

#ifdef GZ_SUPPORT
    gazebo::physics::Link* vehicleLink_ = nullptr;
#endif

    Vector2f goalAreaPosition2D_;

    Vector3f goalAreaPosition3D_;

    FloatType goalAreaRadius_ = 1.0;

    std::unique_ptr<std::uniform_real_distribution<FloatType>> velocityErrorDistribution_ = nullptr;
    std::unique_ptr<std::uniform_real_distribution<FloatType>> yawErrorDistribution_ = nullptr;
    std::unique_ptr<std::uniform_real_distribution<FloatType>> elevationErrorDistribution_ = nullptr;

    std::vector<Tile> tiles_;

    bool updateGazebo = false;

private:
    bool collides_(const geometric::Pose &pose) const {
        auto robotCollisionObject =
            robotEnvironment_->getScene()->getRobotCollisionObjects()[0]->getFCLCollisionObject();
        fcl::Quaternion3f fclQuat(pose.orientation.w(),
                                  pose.orientation.x(),
                                  pose.orientation.y(),
                                  pose.orientation.z());
        fcl::Vec3f translationVector(pose.position.x(),
                                     pose.position.y(),
                                     pose.position.z());
        fcl::Transform3f trans(fclQuat, translationVector);
        robotCollisionObject->setTransform(trans);
        robotCollisionObject->computeAABB();
        CollisionRequest collisionRequest;
        collisionRequest.enableContact = false;
        return robotEnvironment_->getScene()->makeDiscreteCollisionReport(&collisionRequest)->collides;
    }


    void makeTiles() {
        VectorString tileNames({"Tile_1::link", "Tile_2::link", "Tile_3::link"});
        auto bodies = robotEnvironment_->getScene()->getBodies();

        for (auto &tileName : tileNames) {
            bool tileFound = false;
            for (auto &body : bodies) {
                if (body->getScopedName().find(tileName) != std::string::npos) {
                    tileFound = true;
                    auto geom = body->getVisualGeometries()[0];
                    auto dimensions = geom->as<geometric::Box>()->getDimensions();
                    Tile tile;

                    auto tilePose = body->getWorldPose();

                    tile.minX = tilePose.position.x() - dimensions[0] / 2.0;
                    tile.minY = tilePose.position.y() - dimensions[1] / 2.0;
                    tile.minZ = tilePose.position.z() - dimensions[2] / 2.0;

                    tile.maxX = tilePose.position.x() + dimensions[0] / 2.0;
                    tile.maxY = tilePose.position.y() + dimensions[1] / 2.0;
                    tile.maxZ = tilePose.position.z() + dimensions[2] / 2.0;
                    tiles_.push_back(tile);
                }
            }

            if (tileFound == false)
                ERROR("Tile '" + tileName + "' could not be found");
        }
    }

    void makeErrorDistributions() {
        auto options = static_cast<const ParkingTransitionPluginOptions *>(options_.get());
        velocityErrorDistribution_ =
            std::make_unique<std::uniform_real_distribution<FloatType>>(-options->controlErrorVelocity, options->controlErrorVelocity);
        yawErrorDistribution_ =
            std::make_unique<std::uniform_real_distribution<FloatType>>(-options->controlErrorYaw, options->controlErrorYaw);
        elevationErrorDistribution_ =
            std::make_unique<std::uniform_real_distribution<FloatType>>(-options->controlErrorElevation, options->controlErrorElevation);
    }


    RobotStateUserDataSharedPtr makeUserData(const VehicleState *nextState,
            const VectorFloat &action,
            const bool &collides) const {
        RobotStateUserDataSharedPtr userData(new VehicleUserData);
        auto ud = userData->as<VehicleUserData>();
        if (action.size() == 3) {
            ud->insideGoalArea =
                (goalAreaPosition3D_ - Vector3f(nextState->x(), nextState->y(), nextState->z())).norm()
                <= goalAreaRadius_ ? true : false;
        } else {
            ud->insideGoalArea =
                (goalAreaPosition2D_ - Vector2f(nextState->x(), nextState->y())).norm() <= goalAreaRadius_ ? true : false;
        }
        for (int i = 0; i != tiles_.size(); ++i) {
            if (tiles_[i].insideTile(nextState->x(), nextState->y())) {
                ud->aboveTile = i + 1;
                break;
            }
        }

        ud->collides = collides;
        return userData;
    }

    void parseGoalArea() {
        auto bodies = robotEnvironment_->getScene()->getBodies();
        for (auto &body : bodies) {
            if (body->getName() == "GoalArea") {
                auto pose = body->getWorldPose();
                goalAreaPosition2D_ = Vector2f(pose.position.x(), pose.position.y());
                goalAreaPosition3D_ = pose.position;
                goalAreaRadius_ = body->getVisualGeometries()[0]->as<geometric::Sphere>()->getRadius();

            }
        }
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

OPPT_REGISTER_TRANSITION_PLUGIN(ParkingTransitionPlugin)

}
