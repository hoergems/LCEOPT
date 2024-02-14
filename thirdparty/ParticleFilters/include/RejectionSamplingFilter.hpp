#pragma once
#include <oppt/filter/particleFilter/ParticleFilter.hpp>

namespace oppt {
class RejectionSamplingFilter: public ParticleFilter {
public:
	RejectionSamplingFilter();

	virtual ~RejectionSamplingFilter() = default;

	virtual FilterResultPtr propagateParticles(const FilterRequestPtr& filterRequest) override;	

	virtual FilterResultPtr filter(const FilterRequestPtr& filterRequest) override;

private:
	size_t nEffectiveParticles_ = 0;

};

}