#pragma once
#include <oppt/filter/particleFilter/ParticleFilter.hpp>

namespace oppt {
class LowVarianceFilter: public ParticleFilter {
public:
	LowVarianceFilter(const size_t &nEffectiveParticles);

	virtual ~LowVarianceFilter() = default;	

	virtual FilterResultPtr filter(const FilterRequestPtr& filterRequest) override;

private:
	size_t nEffectiveParticles_ = 0;

};

}