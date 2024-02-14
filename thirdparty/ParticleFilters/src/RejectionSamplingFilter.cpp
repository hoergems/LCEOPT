#include "RejectionSamplingFilter.hpp"

namespace oppt {
RejectionSamplingFilter::RejectionSamplingFilter():
	ParticleFilter() {

}

FilterResultPtr RejectionSamplingFilter::propagateParticles(const FilterRequestPtr& filterRequest) {
	auto randomEngine = filterRequest->robotEnvironment->getRobot()->getRandomEngine().get();
	VectorRobotStatePtr nextParticles;
	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	ObservationRequestSharedPtr observationRequest(new ObservationRequest);
	size_t numAttempts = 0;

	std::uniform_int_distribution<int> d(0, filterRequest->previousParticles.size() - 1);

	while (nextParticles.size() < filterRequest->numParticles) {
		propagationRequest->currentState = filterRequest->previousParticles[d(*randomEngine)]->getState();
		propagationRequest->action = filterRequest->action;
		propagationRequest->userData = OpptUserDataSharedPtr(new OpptUserData);
		auto propRes = filterRequest->robotEnvironment->getRobot()->propagateState(propagationRequest);

		observationRequest->currentState = propRes->nextState;
		observationRequest->action = filterRequest->action;
		auto obsRes = filterRequest->robotEnvironment->getRobot()->makeObservationReport(observationRequest);

		// Check if we're terminal
		if (filterRequest->allowTerminalStates) {
			nextParticles.push_back(propRes->nextState);
		} else {
			bool terminal = filterRequest->robotEnvironment->isTerminal(propRes);
			if (terminal == false and filterRequest->observation->equals(*(obsRes->observation.get())))
				nextParticles.push_back(propRes->nextState);
		}

		numAttempts++;
		if (numAttempts > 100 * filterRequest->numParticles) {
			cout << "nextParticles.size(): " << nextParticles.size() << endl;
			if (nextParticles.size() < 0.5 * (FloatType)(filterRequest->numParticles)) {
				WARNING("Giving up");
				return nullptr;
			} else {
				WARNING("Go on with " + std::to_string(nextParticles.size()) + " particles.");
			}

			break;
		}
	}

	FilterResultPtr filterResult(new FilterResult);
	filterResult->particles.resize(nextParticles.size());
	for (size_t i = 0; i != nextParticles.size(); ++i) {
		filterResult->particles[i] = std::make_shared<Particle>(nextParticles[i], 1.0 / ((FloatType)(nextParticles.size())));
	}

	return filterResult;
}

FilterResultPtr RejectionSamplingFilter::filter(const FilterRequestPtr& filterRequest) {
	return propagateParticles(filterRequest);
}
}