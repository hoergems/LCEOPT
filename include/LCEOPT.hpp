#pragma once
#include <oppt/solver/solver.hpp>
#include "ParticleFilters.hpp"
#include "Types.hpp"
#include "ParameterEvaluator.hpp"

using namespace oppt;

namespace solvers {

typedef std::function<ParameterVectors(const unsigned int &)> ParameterSamplingFn;
typedef std::function<void(const Matrixdf &)> UpdateDistributionFn;
typedef std::function<void()> InitDistributionFn;

class LCEOPT: public Solver {
public:
	LCEOPT();

	~LCEOPT() = default;

	void setup() override;

	bool reset() override;

	bool improvePolicy(const FloatType &timeout) override;

	ActionSharedPtr getNextAction() override;

	bool updateBelief(const ActionSharedPtr& action,
	                  const ObservationSharedPtr& observation,
	                  const bool &allowTerminalStates = false) override;

	VectorRobotStatePtr getBeliefParticles() override;

	virtual void stepFinished(const size_t &step) override;

	virtual void serializeStep(std::ofstream& os) override;

private:
	VectorRobotStatePtr beliefParticles_;

	std::unique_ptr<ParticleFilter> particleFilter_ = nullptr;

	std::unique_ptr<ParameterEvaluator> parameterEvaluator_ = nullptr;

	ParameterVectors parameterVectors_;

	ParameterSamplingFn parameterSamplingFn_;

	UpdateDistributionFn updateDistributionFn_;

	InitDistributionFn initDistributionFn_;

	unsigned int numActionDimensions_ = 0;

	unsigned int parameterVectorLength_ = 0;

	unsigned int numEliteSamples_ = 0;

	Vectordf mean_;

	Matrixdf covariance_;

	// For discrete action space
	std::vector<ActionSharedPtr> allActions_;
	VectorFloat weights_;
	std::vector<VectorInt> allParameterVectors_;
	size_t numNodes_;
	std::vector<Matrixdf> probabilityMatrices_;

	VectorFloat lowerActionBound_;
	VectorFloat upperActionBound_;

	FloatType usedPlanningTime_ = 0.0;

private:
	template<class ParameterVectorType>
	void clipParameterVectors(ParamVectors<ParameterVectorType> &parameterVectors) const {
		for (size_t i = 0; i != parameterVectors.cols(); ++i) {
			for (size_t j = 0; j != parameterVectors.rows(); ++j) {
				for (size_t k = 0; k != numActionDimensions_; ++k) {
					if (parameterVectors.col(i)[2 * j + k] < lowerActionBound_[k]) {
						parameterVectors.col(i)[2 * j + k] = lowerActionBound_[k];
					} else if (parameterVectors.col(i)[2 * j + k] > upperActionBound_[k]) {
						parameterVectors.col(i)[2 * j + k] = upperActionBound_[k];
					}
				}
			}
		}
	}

	void initMethodsContinuous_();
};
}