#ifndef _MPC_TYPES_HPP_
#define _MPC_TYPES_HPP_
#include "ParameterVector.hpp"

namespace oppt {
#ifdef LAZY_SAMPLING
#ifdef BAYES_NET
typedef ParameterVectorDynamicBayes ParameterVector;
#else
typedef ParameterVectorDynamic ParameterVector;
#endif
typedef ParamVectors<ParameterVector> ParameterVectors;
typedef std::pair<ParameterVector*, FloatType> ParameterVectorValuePair;
#else
typedef Vectordf ParameterVector;
typedef Matrixdf ParameterVectors;
typedef std::pair<Vectordf, FloatType> ParameterVectorValuePair;
#endif

typedef std::vector<ParameterVector> ParameterVectorList;

class ParameterEvaluator;
typedef std::unique_ptr<ParameterEvaluator> ParameterEvaluatorPtr;
}

#endif