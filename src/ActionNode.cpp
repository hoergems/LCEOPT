#include "ActionNode.hpp"
#include "ObservationEdge.hpp"
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>

namespace oppt {
ActionNode::ActionNode(TreeElement *const parentElement):
	TreeElement(parentElement) {

}

void ActionNode::print() const {

}

TreeElement *ActionNode::getObservationEdge(const int &idx) {
	return (*(getChildren() + idx)).get();
}

void ActionNode::setIdx(const int &idx) {
	idx_ = idx;
}

int ActionNode::getIdx() const {
	return idx_;
}

}