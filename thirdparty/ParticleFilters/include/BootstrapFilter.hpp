#pragma once
#include <oppt/filter/particleFilter/ParticleFilter.hpp>

namespace oppt {
class BootstrapFilter: public ParticleFilter {
public:
	BootstrapFilter(const size_t &nEffectiveParticles);

	virtual ~BootstrapFilter() = default;

	virtual FilterResultPtr propagateParticles(const FilterRequestPtr& filterRequest) override;	

	virtual FilterResultPtr filter(const FilterRequestPtr& filterRequest) override;

private:
	size_t nEffectiveParticles_ = 0;

};

}