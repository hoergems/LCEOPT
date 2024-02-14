#include "LCEOPT.hpp"
#include "LCEOPTOptions.hpp"
#include "MultivariateNormal.hpp"
#include "Utils.hpp"
#include <iomanip>

namespace solvers {
LCEOPT::LCEOPT():
	Solver() {
	solverName_ = "LCEOPT";
}


void LCEOPT::setup() {
	auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);
	numActionDimensions_ = robotPlanningEnvironment_->getRobot()->getActionSpace()->getNumDimensions();

	// Setup the particle filter for belief updates
	if (options->particleFilter == "bootstrap") {
		if (options->nEffectiveParticles == 0) {
			ERROR("'nEffectiveParticles' must be greater than zero");
		}
		particleFilter_ = std::unique_ptr<ParticleFilter>(new BootstrapFilter(options->nEffectiveParticles));
	} else if (options->particleFilter == "lowVariance") {
		if (options->nEffectiveParticles == 0) {
			ERROR("'nEffectiveParticles' must be greater than zero");
		}
		particleFilter_ = std::unique_ptr<ParticleFilter>(new LowVarianceFilter(options->nEffectiveParticles));
	} else if (options->particleFilter == "rejectionSampling") {
		particleFilter_ = std::unique_ptr<ParticleFilter>(new RejectionSamplingFilter());
	} else {
		ERROR("particleFilter not recognized. Must be 'bootstrap', 'lowVariance' or 'rejectionSampling'");
	}

	lowerActionBound_ = VectorFloat();
	upperActionBound_ = VectorFloat();
	robotPlanningEnvironment_->getRobot()->getActionSpace()->getActionLimits()->getLimits()->as<VectorLimitsContainer>()->get(lowerActionBound_,
	        upperActionBound_);

	
	// Determine the size of a parameter vector, based on the number of observations
	// And the policy tree depth
	if (options->numObservationEdges == 1) {
		parameterVectorLength_ = (options->policyTreeDepth + 1.0) * numActionDimensions_;
	} else {
		unsigned int numNodes =
		    (unsigned int)((1.0 - std::pow(options->numObservationEdges, options->policyTreeDepth + 1.0)) / (1.0 - options->numObservationEdges));
		parameterVectorLength_ = numNodes * numActionDimensions_;
	}

	// Make initial mean and covariance of the Multivariate Gaussian distribution
	// over the parameter space
	mean_ = Vectordf::Zero(parameterVectorLength_);
	covariance_ =
	    options->initialStdDev * options->initialStdDev * Matrixdf::Identity(parameterVectorLength_,
	            parameterVectorLength_);

	ACTION_TYPE actionType = ACTION_TYPE::DISCRETE;
	parameterEvaluator_ =
	    ParameterEvaluatorPtr(new ParameterEvaluator(robotPlanningEnvironment_, problemEnvironmentOptions_, heuristicPlugin_.get()));
	numEliteSamples_ = std::floor(options->numParameterVectors * options->numEliteSamplesFactor);
	
	// Init parameter vectors
	ParameterVectorList parameterVectorList;
	parameterVectorList.reserve(options->numParameterVectors);
	for (size_t i = 0; i != options->numParameterVectors; ++i) {
		parameterVectorList.push_back(ParameterVector(robotPlanningEnvironment_->getRobot()->getRandomEngine().get(),
		                              &mean_,
		                              &covariance_,
		                              &lowerActionBound_,
		                              &upperActionBound_,
		                              &numActionDimensions_,
		                              &(options->numObservationEdges)));
	}

	parameterVectors_ = ParameterVectors(parameterVectorList);
	initMethodsContinuous_();
}

void LCEOPT::initMethodsContinuous_() {
	initDistributionFn_ = [this]() {
		auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);
		FloatType initCov = options->initialStdDev * options->initialStdDev;
		mean_.col(0) = Vectordf::LinSpaced(mean_.rows(), 0, 0);
		covariance_.diagonal() = Vectordf::LinSpaced(mean_.rows(), initCov, initCov);
	};

	auto updateDistributionFnDynamic = [this](const Matrixdf & topk) {
		auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);
		Vectordf tildeMean = Vectordf::Zero(topk.cols());
		// Col is the dimension, row is the sample
		for (size_t i = 0; i != topk.cols(); ++i) {
			FloatType numNonNanRows = 0.0;
			for (size_t j = 0; j != topk.rows(); ++j) {
				if (std::isnan(topk(j, i)) == false) {
					tildeMean[i] += topk(j, i);
					numNonNanRows += 1.0;
				}
			}

			if (numNonNanRows > 0) {
				tildeMean[i] /= numNonNanRows;
			}

			FloatType cov = 0.0;
			for (size_t j = 0; j != topk.rows(); ++j) {
				if (std::isnan(topk(j, i)) == false) {
					cov += (topk(j, i) - tildeMean[i]) * (topk(j, i) - tildeMean[i]);
				}
			}

			if (numNonNanRows > 0) {
				cov /= numNonNanRows;
				mean_[i] = (1.0 - options->learningRate) * mean_[i] + options->learningRate * tildeMean[i];
				covariance_(i, i) = (1.0 - options->learningRate) * covariance_(i, i) + options->learningRate * cov;
			}
		}
	};


	updateDistributionFn_ = updateDistributionFnDynamic;
}

bool LCEOPT::reset() {
	auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);
	beliefParticles_ = VectorRobotStatePtr(options->minParticleCount, nullptr);
	for (size_t i = 0; i != options->minParticleCount; ++i) {
		beliefParticles_[i] = robotPlanningEnvironment_->sampleInitialState();
	}

	return true;
}

bool LCEOPT::improvePolicy(const FloatType &timeout) {	
	auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);

	// Make initial distribution
	initDistributionFn_();

	int iteration = 0;
	FloatType covarianceSum = 0.0;
	FloatType avgSamplingTime = 0.0;
	FloatType avgEvalTime = 0.0;
	FloatType avgUpdateTime = 0.0;

	FloatType start = oppt::clock_ms();
	FloatType endTime = oppt::clock_ms() + timeout;
	while (true) {
		auto startSampling = oppt::clock_ms();

		// Initialize empty parameter vectors
		parameterVectors_.init();
		auto endSampling = oppt::clock_ms();
		avgSamplingTime += (endSampling - startSampling) / 1000.0;

		// Evaluate the parameter vectors
		auto startEvaluate = oppt::clock_ms();
		std::vector<ParameterVectorValuePair> evaluatedParameters;
		evaluatedParameters.reserve(options->numParameterVectors);
		for (auto i = 0; i != options->numParameterVectors; ++i) {
			ParameterVector *p = &(parameterVectors_.col(i));
			FloatType parameterValue = parameterEvaluator_->evaluateParameterVector(p, beliefParticles_);
			evaluatedParameters.emplace_back(ParameterVectorValuePair(p, parameterValue));
		}

		auto endEvaluate = oppt::clock_ms();
		avgEvalTime += (endEvaluate - startEvaluate) / 1000.0;

		//Sort the parameter vectors by value in decreasing order
		std::sort(evaluatedParameters.begin(),
		          evaluatedParameters.end(),
		[](const ParameterVectorValuePair & p1, const ParameterVectorValuePair & p2) {
			return p1.second > p2.second;
		});

		// Select the best k parameter vectors
		Matrixdf topk(numEliteSamples_, parameterVectorLength_);
		for (size_t i = 0; i != numEliteSamples_; ++i) {
			topk.row(i) = evaluatedParameters[i].first->toEigenVec();
		}		

		auto startUpdateDistr = oppt::clock_ms();

		// Update distribution over parameters based on the elite samples
		updateDistributionFn_(topk);
		auto endUpdateDistr = oppt::clock_ms();
		avgUpdateTime += (endUpdateDistr - startUpdateDistr) / 1000.0;

		iteration++;

		// Check if planning is finished for the current step
		if ((options->maxNumIterations == 0 and options->maxCovarianceSum == 0) and oppt::clock_ms() >= endTime)
			break;
		if (options->maxNumIterations > 0 and iteration == options->maxNumIterations)
			break;
		if (options->maxCovarianceSum > 0 and covariance_.diagonal().sum() < options->maxCovarianceSum)
			break;
	}

	usedPlanningTime_ = (oppt::clock_ms() - start) / 1000.0;
	std::stringstream ss;
	ss << "elapsed: " << usedPlanningTime_ << endl;
	ss << "num iterations: " << iteration << endl;
	ss << "avgSamplingTime: " << avgSamplingTime / ((FloatType)(iteration)) << endl;
	ss << "avgEvalTime: " << avgEvalTime / ((FloatType)(iteration)) << endl;
	ss << "avgUpdateTime: " << avgUpdateTime / ((FloatType)(iteration)) << endl;
	ss << "std dev: ";
	for (size_t i = 0; i != lowerActionBound_.size(); ++i) {
		ss << sqrt(covariance_.diagonal()[i]) << " ";
	}
	ss << endl;
	PRINT(ss.str());

	return true;
}

ActionSharedPtr LCEOPT::getNextAction() {
	auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);
	VectorFloat actionVec(numActionDimensions_, 0.0);
	for (size_t i = 0; i != numActionDimensions_; ++i) {
		actionVec[i] = mean_[i];
	}

	ActionSharedPtr action(new VectorAction(actionVec));
	return action;


	return nullptr;
}

bool LCEOPT::updateBelief(const ActionSharedPtr& action,
                       const ObservationSharedPtr& observation,
                       const bool &allowTerminalStates) {
	auto options = static_cast<const LCEOPTOptions *>(problemEnvironmentOptions_);
	FilterRequestPtr filterRequest = std::make_unique<FilterRequest>();
	filterRequest->robotEnvironment = robotPlanningEnvironment_;
	filterRequest->allowZeroWeightParticles = false;
	filterRequest->allowTerminalStates = allowTerminalStates;
	filterRequest->numParticles = options->minParticleCount;
	filterRequest->action = action.get();
	filterRequest->observation = observation.get();

	auto randomEngine = robotPlanningEnvironment_->getRobot()->getRandomEngine();
	std::uniform_int_distribution<unsigned int> d(0, beliefParticles_.size() - 1);

	for (size_t i = 0; i != options->minParticleCount; ++i) {
		auto state = beliefParticles_[d(*(randomEngine.get()))];
		filterRequest->previousParticles.push_back(std::make_shared<Particle>(state, 1.0));
	}

	FilterResultPtr filterResult = particleFilter_->filter(filterRequest);
	if (filterResult->particles.empty())
		ERROR("No non-zero weight particles found");

	VectorRobotStatePtr nextParticles(filterResult->particles.size(), nullptr);
	for (size_t i = 0; i != filterResult->particles.size(); ++i) {
		nextParticles[i] = filterResult->particles[i]->getState();		
	}	

	beliefParticles_ = nextParticles;
	return true;
}

VectorRobotStatePtr LCEOPT::getBeliefParticles() {
	return beliefParticles_;
}

void LCEOPT::stepFinished(const size_t &step) {
	
}

void LCEOPT::serializeStep(std::ofstream& os) {
	os << "elapsed: " << usedPlanningTime_ << endl;
}

}