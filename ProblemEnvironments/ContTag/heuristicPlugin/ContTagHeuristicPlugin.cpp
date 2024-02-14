#include <oppt/plugin/Plugin.hpp>
#include "TagState.hpp"

namespace oppt
{
class ContTagHeuristicPlugin: public HeuristicPlugin
{
public:
    ContTagHeuristicPlugin():
        HeuristicPlugin() {
    }

    virtual ~ContTagHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        auto state = heuristicInfo->currentState->as<TagState>();
        if (state->tagged())
            return tagSuccessReward_;

        FloatType dist = (state->robotPosition() - state->targetPosition()).norm();
        FloatType finalDiscount = std::pow(heuristicInfo->discountFactor, dist);
        FloatType qVal = -moveCost_ * (1 - finalDiscount) / (1 - heuristicInfo->discountFactor);
        qVal += finalDiscount * tagSuccessReward_;
        return qVal;
    }

private:
    static constexpr FloatType tagSuccessReward_ = 10.0;

    static constexpr FloatType tagFailPenalty_ = 10.0;

    static constexpr FloatType moveCost_ = 1.0;
};

OPPT_REGISTER_HEURISTIC_PLUGIN(ContTagHeuristicPlugin)

}


