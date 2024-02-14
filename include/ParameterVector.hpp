#pragma once
#include <oppt/opptCore/core.hpp>

namespace oppt {

enum ACTION_TYPE {
	DISCRETE,
	CONTINUOUS
};

enum ParamVectorType {
	DYNAMIC,
	BAYES
};

//namespace solvers {
//	class MPC;
//}

class ParameterVectorDynamic;

class ParamVec {
public:
	ParamVec() = default;

	//friend class solvers::MPC;

	virtual ~ParamVec() = default;

	_NO_COPY_BUT_MOVE(ParamVec)

	virtual FloatType operator()(const int &idx) = 0;

	virtual Vectordf toEigenVec() = 0;

	virtual size_t size() const = 0;

	virtual void init() = 0;

	virtual void setProbabilityMatrices(std::vector<Matrixdf> *probabilityMatrices) {
		probabilityMatrices_ = probabilityMatrices;
	}

protected:
	std::vector<Matrixdf> *probabilityMatrices_ = nullptr;
};

template<class ParameterVectorType>
class ParamVectors {
public:
	ParamVectors(std::vector<ParameterVectorType> &parameterVectors):
		parameterVectors_(std::move(parameterVectors)) {

	}

	ParamVectors() = default;

	~ParamVectors() = default;

	_NO_COPY_BUT_MOVE(ParamVectors<ParameterVectorType>)

	ParameterVectorType &col(const int &idx) {
		return parameterVectors_[idx];
	}

	ParamVectorType getType() const {
		return std::is_same<ParameterVectorType, ParameterVectorDynamic>::value ? ParamVectorType::DYNAMIC : ParamVectorType::BAYES;
	}

	size_t cols() const {
		return parameterVectors_.size();
	}

	size_t rows() const {
		return parameterVectors_[0].size();
	}

	void init() {
		for (auto &parameterVector : parameterVectors_) {
			parameterVector.init();
		}

		///////////////////////////////////////////
		//parameterVectors_[0].print();
		///////////////////////////////////////////
	}

	void setDistribution(Vectordf *mean, Matrixdf *covariance) {
		for (auto &parameterVector : parameterVectors_) {
			parameterVector.setDistribution(mean, covariance);
		}
	}

	void setProbabilityMatrices(std::vector<Matrixdf> *probabilityMatrices) {
		for (auto &parameterVector : parameterVectors_) {
			parameterVector.setProbabilityMatrices(probabilityMatrices);
		}
	}

private:
	std::vector<ParameterVectorType> parameterVectors_;
};

class ParameterVectorDynamic: public ParamVec {
public:
	ParameterVectorDynamic(RandomEngine *randomEngine,
	                       const Vectordf *mean,
	                       const Matrixdf *covariance,
	                       const VectorFloat *lowerActionBound,
	                       const VectorFloat *upperActionBound,
	                       const unsigned int *numActionDimensions,
	                       const unsigned int *numObservationEdges);

	virtual ~ParameterVectorDynamic() = default;

	_NO_COPY_BUT_MOVE(ParameterVectorDynamic)

	virtual FloatType operator()(const int &idx) override;

	virtual Vectordf toEigenVec() override;

	virtual size_t size() const override;

	virtual void init() override;	

protected:
	const VectorFloat *lowerActionBound_;
	const VectorFloat *upperActionBound_;
	const Vectordf *mean_;
	const Matrixdf *covariance_;
	const unsigned int *numActionDimensions_;
	const unsigned int *numObservationEdges_;
	RandomEngine *randomEngine_;
	VectorFloat parameterVector_;
};

}