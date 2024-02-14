#include "LowVarianceFilter.hpp"

namespace oppt {
LowVarianceFilter::LowVarianceFilter(const size_t &nEffectiveParticles):
	ParticleFilter(),
	nEffectiveParticles_(nEffectiveParticles) {

}

FilterResultPtr LowVarianceFilter::filter(const FilterRequestPtr& filterRequest) {
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

	for (size_t i = 0; i != filterResult->particles.size(); ++i) {
		FloatType newWeight = filterResult->particles[i]->getWeight();
		FloatType pdf = 1.0;
		if (filterRequest->observation) {
			pdf = robot->calcLikelihood(filterResult->particles[i]->getState(),
			                            filterRequest->action,
			                            filterRequest->observation);
		} else {
			WARNING("No observation");
		}

		newWeight *= pdf;
		if (newWeight > 0 || filterRequest->allowZeroWeightParticles) {
			replenishedParticlesVec[particleCounter] = filterResult->particles[i];
			replenishedParticlesVec[particleCounter]->setWeight(newWeight);
			weightNormalizationConstant += newWeight;
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

	// Normalize weights
	FloatType sumSquaredWeights = 0.0;
	for (size_t i = 0; i != replenishedParticlesVec.size(); ++i) {
		FloatType normalizedWeight = replenishedParticlesVec[i]->getWeight() / weightNormalizationConstant;;
		sumSquaredWeights += normalizedWeight * normalizedWeight;
		replenishedParticlesVec[i]->setWeight(normalizedWeight);
	}

	FloatType n_eff = 1.0 / sumSquaredWeights;	

	if (n_eff < nEffectiveParticles_) {
		// Resampling
		FloatType M = (FloatType)(replenishedParticlesVec.size());
		FloatType r = std::uniform_real_distribution<FloatType>(0, 1.0 / M)(*(randomEngine.get()));
		FloatType c = replenishedParticlesVec[0]->getWeight();

		VectorParticles newParticles;

		int i = 0;
		for (int m = 1; m != replenishedParticlesVec.size() + 1; ++m) {
			auto U = r + ((FloatType)(m) - 1.0) * (1.0 / M);
			while (U > c) {
				i = i + 1;
				c = c + replenishedParticlesVec[i]->getWeight();
			}

			newParticles.emplace_back(replenishedParticlesVec[i]);
		}

		filterResult->particles = newParticles;
		for (size_t i = 0; i != filterResult->particles.size(); ++i) {
			filterResult->particles[i]->setWeight(1.0/(filterResult->particles.size()));
		}
	} else {
		filterResult->particles = replenishedParticlesVec;
	}

	return std::move(filterResult);
}
}