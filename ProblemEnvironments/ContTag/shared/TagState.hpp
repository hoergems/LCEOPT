#pragma once
#include <oppt/opptCore/core.hpp>
#include <oppt/robotHeaders/RobotState.hpp>
#include "Types.hpp"

namespace oppt {
class TagState: public VectorState {
public:
	TagState(const Position &robotPosition, const FloatType &robotAngle, const Position &targetPosition, const bool &tagged):
		VectorState( {robotPosition.x(), robotPosition.y(), robotAngle, targetPosition.x(), targetPosition.y(), tagged == true ? 1.0 : 0.0}),
	             robotPosition_(robotPosition),
	             robotAngle_(robotAngle),
	             targetPosition_(targetPosition),
	tagged_(tagged) {

	}

	virtual ~TagState() = default;

	Vector2f robotPosition() const {
		return robotPosition_;
	}	

	Vector2f targetPosition() const {
		return targetPosition_;
	}	

	FloatType robotAngle() const {
		return robotAngle_;
	}	

	bool tagged() const {
		return tagged_;
	}
	
	virtual void print(std::ostream& os) const override {
		os << "robot: (" << robotPosition_.x() << ", " << robotPosition_.y() << "), target: ("
		   << targetPosition_.x() << ", " << targetPosition_.y() << "), angle: "
		   << robotAngle_ << ", tagged: " << tagged_;
	}

private:
	Position robotPosition_;
	Position targetPosition_;
	FloatType robotAngle_;
	bool tagged_;


};
}