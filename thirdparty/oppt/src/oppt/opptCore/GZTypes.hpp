#pragma once
#include "typedefs_basic.hpp"
#include "oppt/opptCore/UID.hpp"
#ifdef GZ_GT_7
#include <ignition/math4/ignition/math.hh>
#else
#include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>

namespace gazebo {
namespace physics {
class WorldState;
}
}

namespace oppt {

#ifdef GZ_GT_7
typedef ignition::math::Pose3d GZPose;
typedef ignition::math::Vector3<FloatType> GZVector3;
typedef ignition::math::Angle GZAngle;
typedef ignition::math::Quaternion<FloatType> GZQuaternion;
#ifdef GZ_GT_10
typedef ignition::math::AxisAlignedBox GZBox;
#else
typedef ignition::math::Box GZBox;
#endif
#else
typedef gazebo::math::Pose GZPose;
typedef gazebo::math::Vector3 GZVector3;
typedef gazebo::math::Angle GZAngle;
typedef gazebo::math::Quaternion GZQuaternion;
typedef gazebo::math::Box GZBox;
#endif

/** @brief Pointer to a Gazebo simulated world */
typedef gazebo::physics::WorldPtr WorldPtr;
typedef gazebo::physics::ModelPtr ModelPtr;
typedef gazebo::physics::LinkPtr LinkPtr;
typedef gazebo::physics::JointPtr JointPtr;

/** @brief std::shared_ptr to a gazebo::physics::WorldState */
typedef std::shared_ptr<gazebo::physics::WorldState> WorldStateSharedPtr;

/** @brief std::unique_ptr to a gazebo::physics::WorldState */
typedef std::unique_ptr<gazebo::physics::WorldState> WorldStateUniquePtr;

class GazeboWorldState;
typedef std::shared_ptr<GazeboWorldState> GazeboWorldStatePtr;

class GazeboInterface;
typedef std::shared_ptr<GazeboInterface> GazeboInterfaceSharedPtr;

/**
 * A wrapper class around a gazebo::physics::WorldState
 * This class represents the complete state of an environment
 */
class GazeboWorldState
{
public:
    _NO_COPY_BUT_MOVE(GazeboWorldState)

    /**
     * @brief Construct from WorldPtr
     */
    GazeboWorldState(const WorldPtr& world):
        world_(world),
        worldState_(std::make_unique<gazebo::physics::WorldState>(world)),
        id_(UID::getUniqueId()),
        cumulativeAngles_() {

    }

    /**
     * @brief Construct from a WorldStatePtr and a WorldPtr
     */
    GazeboWorldState(WorldStateUniquePtr worldState, const WorldPtr& world):
        world_(world),
        worldState_(std::move(worldState)),
        id_(UID::getUniqueId()),
        cumulativeAngles_() {

    }

    /** @brief Default destructor */
    virtual ~GazeboWorldState() = default;

    /**
     * @brief Get a WorldPtr to the underlying Gazebo world
     */
    WorldPtr getWorld() const {
        return world_;
    }

    /**
     * @brief Get a WorldStatePtr to the underlying gazebo::physics::WorldState
     */
    gazebo::physics::WorldState* getWorldState() const {
        return worldState_.get();
    }

    /**
     * @brief Update the world state
     */
    void updateWorldState() {
        worldState_ = std::make_unique<gazebo::physics::WorldState>(world_);
    }

    /**
     * @brief Get the ID of this world state
     */
    const long getId() const {
        return id_;
    }

    /**
     * @brief Get a string representation of this world state
     */
    std::string toString() const {
#ifdef GZ_GT_7
        const sdf::ElementPtr worldSDF =
            world_->SDF();
        sdf::ElementPtr stateElem = worldSDF->GetElement("state");
        worldState_->FillSDF(stateElem);
        return stateElem->ToString("");
#else
        return "";
#endif
    }

    /**
     * @brief Compares two GazeboWorldState objects using their IDs
     */
    friend bool operator== (const GazeboWorldState& s1, const GazeboWorldState& s2) {
        return s1.getId() == s2.getId();
    }

    /**
     * @brief Compares two GazeboWorldState objects using their IDs
     */
    friend bool operator!= (const GazeboWorldState& s1, const GazeboWorldState& s2) {
        return !(s1.getId() == s2.getId());
    }

    void setCumulativeAngles(const CumulativeAnglesVec& cumulativeAngles) {
        cumulativeAngles_ = cumulativeAngles;
    }

    const CumulativeAnglesVec getCumulativeAngles() const {
        return cumulativeAngles_;
    }

private:
    WorldPtr world_;

    WorldStateUniquePtr worldState_;

    CumulativeAnglesVec cumulativeAngles_;

    const long id_;
};
}
