#pragma once
#include <oppt/opptCore/core.hpp>

namespace oppt {
void ToTree(const std::vector<Matrixdf> probabilityMatrices, const int &numObservationEdges, const int maxDepth = -1);
void ToTree(const Matrixdf &covariance, const int &numObservationEdges, const int maxDepth = -1);
}