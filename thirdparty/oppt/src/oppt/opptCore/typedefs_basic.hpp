#pragma once
#include "oppt/defs.hpp"

namespace oppt {
/** @brief A std::vector of oppt::FloatType */
typedef std::vector<FloatType> VectorFloat;
/** @brief A std::vector of strings */
typedef std::vector<std::string> VectorString;
/** @brief A std::vector of unsigned integers */
typedef std::vector<unsigned int> VectorUInt;
/** @brief A std::vector of signed integers */
typedef std::vector<int> VectorInt;

typedef std::vector<std::pair<std::string, VectorFloat>> CumulativeAnglesVec;
}