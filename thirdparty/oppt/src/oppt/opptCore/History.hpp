#pragma once
#include "typedefs.hpp"

namespace oppt {
struct HistoryEntry {
	const Action *action = nullptr;
	const Observation *observation = nullptr;
};
}