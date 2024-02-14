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
#ifndef ___ROBOT_STATE_HPP___
#define ___ROBOT_STATE_HPP___
#include "includes.hpp"
#include "oppt/opptCore/UID.hpp"
#include "RobotStateUserData.hpp"

namespace oppt
{

using std::cout;
using std::endl;

extern bool logGazeboState;
inline void enableGazeboStateLogging(const bool& logging) {
    logGazeboState = logging;
}

//#ifdef GZ_SUPPORT

//#endif

/**
 * A class that represents a single state of the robot.
 */
class RobotState: public OpptUserData
{
public:
    using OpptUserData::OpptUserData;

    _NO_COPY_BUT_MOVE(RobotState)

    RobotState() = default;

    virtual ~RobotState() = default;

    _STATIC_CAST    

    /**
     * @brief Print the state to an output stream
     */
    virtual void print(std::ostream& os) const = 0;

    /**
     * @brief Serialize the state to an output stream.
     */
    virtual void serialize(std::ostream& os, const std::string prefix = "") const {
        if (prefix != "")
            os << prefix << ": ";
        print(os);
    }

    friend std::ostream& operator<< (std::ostream& out, const RobotState& robotState) {
        robotState.print(out);
        return out;
    }

    /**
     * @brief Checks if this state is equal to another state
     * @param otherState The other state
     *
     * @return True if this state equals otherState
     */
    virtual bool equals(const RobotState& otherState) const = 0;

    /**
     * @brief Calculates the distance of this state to another state
     * @param otherState The other state
     *
     * @return The distance between this state and the other state
     */
    virtual FloatType distanceTo(const RobotState& otherState) const = 0;

    /**
     * @brief Calculates a hash value of this state
     */
    virtual std::size_t hash() const = 0;

#ifdef GZ_SUPPORT
    /**
     * @brief Get a pointer to the underlying GazeboWorldState
     */
    GazeboWorldStatePtr getGazeboWorldState() const {
        return gazeboWorldState_;
    }

    /**
     * @brief Set the underlying GazeboWorldState
     */
    void setGazeboWorldState(const GazeboWorldStatePtr& worldState) {
        gazeboWorldState_ = worldState;
    }
#endif

    void setSubStates(const std::vector<RobotStateSharedPtr>& subStates);

    const VectorRobotStatePtr getSubStates() const;

    /**
     * @brief Set additional user data
     */
    void setUserData(const OpptUserDataSharedPtr& userData) {
        userData_ = userData;
    }

    /**
     * @brief Get the additional user data
     */
    const OpptUserDataSharedPtr getUserData() const {
        return userData_;
    }

    /**
     * @brief Clear the user data
     */
    void clearUserData() {
        userData_.reset();
    }


protected:
#ifdef GZ_SUPPORT
    GazeboWorldStatePtr gazeboWorldState_;
#endif

    std::vector<RobotStateSharedPtr> subStates_;

    OpptUserDataSharedPtr userData_ = nullptr;

};

}

#endif
