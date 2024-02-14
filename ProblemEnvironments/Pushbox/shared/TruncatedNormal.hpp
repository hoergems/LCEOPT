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
		PhiAlpha = phi((lowerBound - mean) / stddev_);
		FloatType PhiBeta = phi((upperBound - mean) / stddev_);
		Z = PhiBeta - PhiAlpha;		

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

	FloatType pdf(const FloatType &x) const {
		if (x < lowerBound or x > upperBound)
			return 0.0;

		return (1.0 / stddev_) * (psi((x - mean) / stddev_)) / Z;
	}

	FloatType cdf(const FloatType &x) const {
		if (x >= upperBound)
			return 1.0;
		if (x <= lowerBound)
			return 0.0;
		return (phi((x-mean)/stddev_) - PhiAlpha) / Z;
	}

	FloatType getLowerBound() const {
		return lowerBound;
	}

	FloatType getUpperBound() const {
		return upperBound;
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

	static constexpr FloatType sqrtTwo = sqrt(2.0);
	FloatType PhiAlpha = 0.0;
	FloatType Z = 0.0;

private:
	FloatType psi(const FloatType &x) const {
		static constexpr FloatType invSqrt2Pi = 1.0 / sqrt(2.0 * M_PI);
		return invSqrt2Pi * exp(-0.5 * x * x);
	}

	FloatType phi(const FloatType &x) const {
		return 0.5 * (1.0 + std::erf(x / sqrtTwo));
	}

};

typedef std::unique_ptr<TruncatedNormalDistribution> TruncatedNormalPtr;

}

#endif