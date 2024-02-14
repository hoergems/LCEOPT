#ifndef _SENSOR_PLACEMENT_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _SENSOR_PLACEMENT_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class SensorPlacementTransitionPluginOptions: public PluginOptions
{
public:
    SensorPlacementTransitionPluginOptions() = default;

    virtual ~SensorPlacementTransitionPluginOptions() = default;

    VectorFloat lowerTransitionErrorBound;

    VectorFloat upperTransitionErrorBound;

    VectorString jointNames;

    std::string manipulatorURDF = "";

    unsigned int particlePlotLimit = 0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPegInHoleTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addPegInHoleTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<VectorFloat>("transitionPluginOptions", "lowerTransitionErrorBound", &SensorPlacementTransitionPluginOptions::lowerTransitionErrorBound);
        parser->addOption<VectorFloat>("transitionPluginOptions", "upperTransitionErrorBound", &SensorPlacementTransitionPluginOptions::upperTransitionErrorBound);
        parser->addOption<VectorString>("state", "jointPositions", &SensorPlacementTransitionPluginOptions::jointNames);
        parser->addOption<std::string>("transitionPluginOptions", "manipulatorURDF", &SensorPlacementTransitionPluginOptions::manipulatorURDF);
        parser->addOption<unsigned int>("simulation", "particlePlotLimit", &SensorPlacementTransitionPluginOptions::particlePlotLimit);

    }

};
}

#endif
