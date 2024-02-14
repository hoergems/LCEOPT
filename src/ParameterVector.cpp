#include "ParameterVector.hpp"

namespace oppt {
ParameterVectorDynamic::ParameterVectorDynamic(RandomEngine *randomEngine,
        const Vectordf *mean,
        const Matrixdf *covariance,
        const VectorFloat *lowerActionBound,
        const VectorFloat *upperActionBound,
        const unsigned int *numActionDimensions,
        const unsigned int *numObservationEdges):
	ParamVec(),
	randomEngine_(randomEngine),
	mean_(mean),
	covariance_(covariance),
	parameterVector_(VectorFloat(covariance_->rows(), std::numeric_limits<FloatType>::quiet_NaN())),
	lowerActionBound_(lowerActionBound),
	upperActionBound_(upperActionBound),
	numActionDimensions_(numActionDimensions),
	numObservationEdges_(numObservationEdges) {

}

FloatType ParameterVectorDynamic::operator()(const int &nodeIdx) {
	// Get the action associated to the node with index 'nodeIdx'.
	// If there's no associated action yet (parameterVector_[nodeIdx] == nan), sample one
	if (std::isnan(parameterVector_[nodeIdx])) {
		parameterVector_[nodeIdx] = std::normal_distribution<FloatType>((*mean_)[nodeIdx], sqrt((*covariance_)(nodeIdx, nodeIdx)))(*randomEngine_);
		int actionDimensionIdx = nodeIdx % (*numActionDimensions_);
		if (parameterVector_[nodeIdx] < lowerActionBound_->operator[](actionDimensionIdx)) {
			parameterVector_[nodeIdx] = lowerActionBound_->operator[](actionDimensionIdx);
		} else if (parameterVector_[nodeIdx] > upperActionBound_->operator[](actionDimensionIdx)) {
			parameterVector_[nodeIdx] = upperActionBound_->operator[](actionDimensionIdx);
		}
	}

	return parameterVector_[nodeIdx];
}


Vectordf ParameterVectorDynamic::toEigenVec() {
	Vectordf v2 = Eigen::Map<Vectordf, Eigen::Unaligned>(parameterVector_.data(), parameterVector_.size());
	return v2;
}

size_t ParameterVectorDynamic::size() const {
	return parameterVector_.size();
}

void ParameterVectorDynamic::init() {
	for (size_t i = 0; i != parameterVector_.size(); ++i) {
		parameterVector_[i] = std::numeric_limits<FloatType>::quiet_NaN();
	}
}

}