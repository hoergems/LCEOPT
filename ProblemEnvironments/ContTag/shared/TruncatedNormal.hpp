#ifndef _TRUNCATED_NORMAL_HPP_
#define _TRUNCATED_NORMAL_HPP_
#include <oppt/opptCore/core.hpp>

namespace oppt {
class TruncatedNormalDistribution {
public:
	TruncatedNormalDistribution(const FloatType theMean = 0.0,
	                            const FloatType stddev = 1.0):
		mean(theMean),
		stddev_(stddev),
		lowerBound(mean - stddev),
		upperBound(mean + stddev),
		zeroStdDev(stddev == 0),
		normalDistribution(mean, stddev) {

	}

	~TruncatedNormalDistribution() = default;

	_NO_COPY_BUT_MOVE(TruncatedNormalDistribution)

	FloatType sample(RandomEngine &randomEngine) {
		if (zeroStdDev) return lowerBound;
		while (true) {
			FloatType result = normalDistribution(randomEngine);
			if ( (result >= lowerBound) && (result <= upperBound) ) {
				return result;
			}
		}
	}

	FloatType pdf(const FloatType &value) const {
		if (value < lowerBound or value > upperBound)
			return 0.0;
		static constexpr FloatType invSqrt2Pi = 1.0 / sqrt(2.0*M_PI);
		FloatType a = value / stddev_;
		return invSqrt2Pi * exp(-0.5 * a * a);
	}

	bool hasZeroStdDev() const { return zeroStdDev; }
	FloatType getMean() const { return mean; }
private:
	FloatType mean;
	FloatType stddev_;
	FloatType lowerBound;
	FloatType upperBound;
	bool zeroStdDev;
	std::normal_distribution<FloatType> normalDistribution;
};

typedef std::unique_ptr<TruncatedNormalDistribution> TruncatedNormalPtr;

}

#endif