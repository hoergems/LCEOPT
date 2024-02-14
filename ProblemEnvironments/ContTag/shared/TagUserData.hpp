#pragma once
#include <oppt/opptCore/core.hpp>

namespace oppt {
class TagUserData: public RobotStateUserData {
public:
	using RobotStateUserData::RobotStateUserData;

	bool movedIntoWall = false;

};
}

