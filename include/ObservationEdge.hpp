#ifndef _OBSERVATION_EDGE_HPP_
#define _OBSERVATION_EDGE_HPP_
#include "TreeElement.hpp"

namespace oppt {
class ObservationEdge: public TreeElement {
public:
	ObservationEdge(TreeElement *const parentElement);

	virtual void print() const;

	void setObservation(ObservationSharedPtr &observation);

	ObservationSharedPtr getObservation() const;

private:
	ObservationSharedPtr observation_ = nullptr;
};
}

#endif