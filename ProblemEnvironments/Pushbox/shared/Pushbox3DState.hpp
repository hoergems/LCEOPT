#pragma once
#include <oppt/opptCore/core.hpp>
#include <oppt/robotHeaders/RobotState.hpp>

namespace oppt {
typedef Vector3f Position;

class Pushbox3DState: public VectorState {
public:
	Pushbox3DState(const Position &robotPosition, const Position &targetPosition):
		VectorState( {robotPosition.x(), robotPosition.y(), robotPosition.z(), targetPosition.x(), targetPosition.y(), targetPosition.z()}),
	             robotPosition_(robotPosition),
	targetPosition_(targetPosition) {

	}

	Pushbox3DState(const VectorFloat &stateVec):
		VectorState(stateVec),
		robotPosition_(Position(stateVec[0], stateVec[1], stateVec[2])),
		targetPosition_(Position(stateVec[3], stateVec[4], stateVec[5])) {

	}

	virtual ~Pushbox3DState() = default;

	Position robotPosition() const {
		return robotPosition_;
	}

	Position targetPosition() const {
		return targetPosition_;
	}

private:
	Position robotPosition_;
	Position targetPosition_;
};
}