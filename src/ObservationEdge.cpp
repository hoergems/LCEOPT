#include "ObservationEdge.hpp"

namespace oppt {
ObservationEdge::ObservationEdge(TreeElement *const parentElement):
	TreeElement(parentElement) {

}

void ObservationEdge::print() const {

}

void ObservationEdge::setObservation(ObservationSharedPtr &observation) {
	observation_ = observation;
}

ObservationSharedPtr ObservationEdge::getObservation() const {
	return observation_;
}

}