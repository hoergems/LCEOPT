#include "oppt/plugin/Plugin.hpp"
#include "TagState.hpp"
#include "TagUserData.hpp"

namespace oppt
{
class ContTagRewardPlugin: public RewardPlugin
{
public :
    ContTagRewardPlugin():
        RewardPlugin() {
    }

    virtual ~ContTagRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        auto state = propagationResult->nextState->as<TagState>();
        auto action = propagationResult->action->as<VectorAction>()->asVector();
        if (state->tagged()) {
            return tagSuccessReward_;
        }

        if (action[1] > 0.5) {
            // Tag action
            return -tagFailPenalty_;
        }     

        return -moveCost_;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-6.0,
                              100.0);
    }

private:
    static constexpr FloatType tagSuccessReward_ = 10.0;

    static constexpr FloatType tagFailPenalty_ = 10.0;

    static constexpr FloatType moveCost_ = 1.0;
};

OPPT_REGISTER_REWARD_PLUGIN(ContTagRewardPlugin)

}
