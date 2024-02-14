#ifndef _CONT_TAG_TYPES_HPP_
#define _CONT_TAG_TYPES_HPP_
#include <oppt/opptCore/core.hpp>
#include "map2d.hpp"

namespace oppt {
typedef Vector2f Position;
typedef Eigen::Matrix<FloatType, Eigen::Dynamic, 2> Positions;

typedef Map2D<char> Map;

typedef std::uniform_int_distribution<int> IntDistribution;
typedef std::unique_ptr<IntDistribution> IntDistributionPtr;

typedef std::uniform_real_distribution<FloatType> UniformRealDistribution;
typedef std::unique_ptr<UniformRealDistribution> UniformRealDistributionPtr;
}

#endif