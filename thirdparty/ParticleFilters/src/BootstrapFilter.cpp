#include "BootstrapFilter.hpp"

namespace oppt {
BootstrapFilter::BootstrapFilter(const size_t &nEffectiveParticles):
	ParticleFilter(),
	nEffectiveParticles_(nEffectiveParticles) {

}

FilterResultPtr BootstrapFilter::propagateParticles(const FilterRequestPtr& filterRequest) {	
	PropagationRequestSharedPtr propagationRequest(new PropagationRequest);
	propagationRequest->action = filterRequest->action;
	propagationRequest->userData = OpptUserDataSharedPtr(new ParticleFilterUserData);
	FilterResultPtr filterResult(new FilterResult);
	filterResult->particles.reserve(filterRequest->numParticles);
	auto randomEngine = filterRequest->robotEnvironment->getRobot()->getRandomEngine().get();

	// Collect the current weights
	VectorFloat weights(filterRequest->previousParticles.size(), 0.0);
	for (size_t i = 0; i != weights.size(); ++i) {
		weights[i] = filterRequest->previousParticles[i]->getWeight();
	}

	std::discrete_distribution<int> distr(weights.begin(), weights.end());

	for (size_t i = 0; i != filterRequest->numParticles; ++i) {
		// Sample particle
		propagationRequest->currentState = filterRequest->previousParticles[distr(*randomEngine)]->getState();

		// Propagate particle
		auto propagationResult = filterRequest->robotEnvironment->getRobot()->propagateState(propagationRequest);
		if (filterRequest->robotEnvironment->isTerminal(propagationResult) == false or filterRequest->allowTerminalStates) {
			filterResult->particles.emplace_back(std::make_shared<Particle>(propagationResult->nextState, 0.0));
		}
	}

	return filterResult;
}

FilterResultPtr BootstrapFilter::filter(const FilterRequestPtr& filterRequest) {
	auto robot = filterRequest->robotEnvironment->getRobot();
	auto randomEngine = robot->getRandomEngine();
	long currNumNextParticles = filterRequest->currentNextParticles.size();
	FilterResultPtr filterResult = propagateParticles(filterRequest);
	VectorParticles replenishedParticlesVec(filterResult->particles.size());
	if (replenishedParticlesVec.empty()) {
		WARNING("Couldn't generate any valid particles in ParticleFilter. Check your TerminalPlugin");
	}
	FloatType weightNormalizationConstant = 0.0;
	size_t particleCounter = 0;

	FloatType weightConstant = 1.0 / ((FloatType)(filterResult->particles.size()));
	for (size_t i = 0; i != filterResult->particles.size(); ++i) {
		//FloatType newWeight = filterResult->particles[i]->getWeight();
		FloatType pdf = weightConstant;
		if (filterRequest->observation) {
			pdf = robot->calcLikelihood(filterResult->particles[i]->getState(),
			                            filterRequest->action,
			                            filterRequest->observation);
		} else {
			WARNING("No observation");
		}
		
		if (pdf > 0 || filterRequest->allowZeroWeightParticles) {
			replenishedParticlesVec[particleCounter] = filterResult->particles[i];
			replenishedParticlesVec[particleCounter]->setWeight(pdf);
			weightNormalizationConstant += pdf;
			particleCounter++;
		}
	}

	if (particleCounter == 0) {
		// We couldn't sample any particles
		filterResult->particles.resize(0);
		WARNING("All subsequent particles have 0 weight. Check our calcLikelihood function in your ObservationPlugin");
		return filterResult;
	}

	replenishedParticlesVec.resize(particleCounter);

	// Normalize the weights
	FloatType sumSquaredWeights = 0.0;
	for (size_t i = 0; i != replenishedParticlesVec.size(); ++i) {
		FloatType normalizedWeight = replenishedParticlesVec[i]->getWeight() / weightNormalizationConstant;;
		sumSquaredWeights += normalizedWeight * normalizedWeight;
		replenishedParticlesVec[i]->setWeight(normalizedWeight);
	}

	FloatType n_eff = 1.0 / sumSquaredWeights;
	if (n_eff < nEffectiveParticles_) {		
		// Resampling
		LOGGING("RESAMPLING")
		filterResult->particles.resize(filterRequest->numParticles);
		VectorFloat weights(replenishedParticlesVec.size(), 0.0);
		for (size_t i = 0; i != replenishedParticlesVec.size(); ++i) {
			weights[i] = replenishedParticlesVec[i]->getWeight();
		}

		std::discrete_distribution<int> distr(weights.begin(), weights.end());
		FloatType weightConstant = 1.0 / ((FloatType)(filterRequest->numParticles));
		for (size_t i = 0; i != filterRequest->numParticles; ++i) {
			filterResult->particles[i] = replenishedParticlesVec[distr(*(randomEngine.get()))];
			filterResult->particles[i]->setWeight(weightConstant);
		}


		/**LOGGING("RESAMPLING");
		ParticleSet replenishedParticles;
		replenishedParticles.setParticles(replenishedParticlesVec);
		auto resampledParticles = replenishedParticles.sampleWeighted(randomEngine, filterRequest->numParticles);
		FloatType newWeight = 1.0 / ((FloatType)resampledParticles.size());
		filterResult->particles.resize(filterRequest->numParticles);
		for (size_t i = 0; i != resampledParticles.size(); ++i) {
			filterResult->particles[i] = resampledParticles[i];
			filterResult->particles[i]->setWeight(newWeight);
		}*/

	} else {
		// No resampling
		LOGGING("NO RESAMPLING");
		filterResult->particles = replenishedParticlesVec;
	}	

	return std::move(filterResult);
}
}