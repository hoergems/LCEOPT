#ifndef _ACTION_NODE_HPP_
#define _ACTION_NODE_HPP_
#include "TreeElement.hpp"

namespace oppt {
class ActionNode: public TreeElement {
public:
	ActionNode(TreeElement *const parentElement);

	virtual void print() const override;

	void setIdx(const int &idx);

	int getIdx() const;

	TreeElement *getObservationEdge(const int &idx);

private:
	int idx_ = 0;	
};
}
#endif