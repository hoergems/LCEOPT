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
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "oppt/plugin/Plugin.hpp"
#include "SensorPlacementTransitionPluginOptions.hpp"
#include "GoalParser.hpp"
#include "SensorPlacementUserData.hpp"

#ifdef GZ_SUPPORT
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#endif

namespace oppt
{

typedef pinocchio::JointIndex JointIndex;

typedef pinocchio::Model KinematicModel;
typedef std::unique_ptr<pinocchio::Model> KinematicModelPtr;

typedef pinocchio::Data Data;
typedef std::unique_ptr<Data> DataPtr;


class SensorPlacementTransitionPluginKin: public TransitionPlugin
{
public :
    SensorPlacementTransitionPluginKin():
        TransitionPlugin() {

    }

    virtual ~SensorPlacementTransitionPluginKin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SensorPlacementTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const SensorPlacementTransitionPluginOptions *>(options_.get());
        parseGoal_();
        auto re = robotEnvironment_->getRobot()->getRandomEngine();

        if (resources::FileExists(options->manipulatorURDF) == false)
            ERROR("'" + options->manipulatorURDF + "' doesn't exist")
        std::string urdfFile = resources::FindFile(options->manipulatorURDF);

        transitionErrorDistribution_ =
            std::make_unique<UniformDistribution<FloatType>>(options->lowerTransitionErrorBound, options->upperTransitionErrorBound, re);
        model_ = KinematicModelPtr(new pinocchio::Model);
        pinocchio::urdf::buildModel(urdfFile, *(model_.get()));
        data_ = DataPtr(new Data(*(model_.get())));
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        PropagationResultSharedPtr propRes(new PropagationResult);
        VectorFloat currentStateVec = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat nextStateVec = oppt::addVectors(currentStateVec, propagationRequest->action->as<VectorAction>()->asVector());
        Vectordf q = toEigenVec(nextStateVec);        
        pinocchio::framesForwardKinematics(*(model_.get()), *(data_.get()), q);        
        propRes->nextState = RobotStateSharedPtr(new VectorState(nextStateVec));
#ifdef GZ_SUPPORT
        if (robotEnvironment_->isExecutionEnvironment() or
                (propagationRequest->userData != nullptr and
                 static_cast<const SensorPlacementTransitionPluginOptions *>(options_.get())->particlePlotLimit > 0)) {
            robotEnvironment_->getGazeboInterface()->setStateVector(nextStateVec);
            propRes->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        }
#endif
        
        propRes->nextState->setUserData(makeUserData_());
        return propRes;
    }

private:
    Vector3f goalPosition_;

    FloatType goalRadius_ = 0.0;

    std::unique_ptr<UniformDistribution<FloatType>> transitionErrorDistribution_ = nullptr;    

    KinematicModelPtr model_ = nullptr;

    DataPtr data_ = nullptr;

private:
    OpptUserDataSharedPtr makeUserData_() const {
        OpptUserDataSharedPtr userData(new SensorPlacementUserData);

        Vector3f eePosition = data_->oMf[data_->oMf.size() - 1].translation();
        userData->as<SensorPlacementUserData>()->collides = false;
        userData->as<SensorPlacementUserData>()->eeTouchesWall = false;
        userData->as<SensorPlacementUserData>()->eePose = geometric::Pose(eePosition.x(), eePosition.y(), eePosition.z(), 0.0, 0.0, 0.0);

        if (std::fabs(goalPosition_.x() - eePosition.x()) < 0.15)
            userData->as<SensorPlacementUserData>()->eeTouchesWall = true;
        if (eePosition.x() > goalPosition_.x() + 0.1)
            userData->as<SensorPlacementUserData>()->collides = true;

        if (robotEnvironment_->isExecutionEnvironment()) {
            cout << "collides: " << userData->as<SensorPlacementUserData>()->collides << endl;
            cout << "touchesWall: " << userData->as<SensorPlacementUserData>()->eeTouchesWall << endl;
        }

        // Check if we reached the goal
        userData->as<SensorPlacementUserData>()->reachedGoal = (goalPosition_ - eePosition).norm() <= goalRadius_ ? true : false;
        return userData;
    }

    void parseGoal_() {
        GoalParser p;
        VectorFloat goalArea = p.parseGoalAreaFromFile(robotEnvironment_->getWorldFile());
        goalPosition_ = Vector3f(goalArea[0], goalArea[1], goalArea[2]);
        goalRadius_ = goalArea[3];
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(SensorPlacementTransitionPluginKin)

}
