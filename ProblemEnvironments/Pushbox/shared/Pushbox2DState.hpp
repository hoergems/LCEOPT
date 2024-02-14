#pragma once
#include <oppt/opptCore/core.hpp>
#include <oppt/robotHeaders/RobotState.hpp>

namespace oppt {
typedef Vector2f Position;

class Pushbox2DState: public VectorState {
public:
	Pushbox2DState(const Position &robotPosition, const Position &targetPosition):
		VectorState( {robotPosition.x(), robotPosition.y(), targetPosition.x(), targetPosition.y()}),
	             robotPosition_(robotPosition),
	targetPosition_(targetPosition) {

	}

	virtual ~Pushbox2DState() = default;

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