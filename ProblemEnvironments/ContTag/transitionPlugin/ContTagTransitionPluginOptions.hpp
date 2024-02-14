#pragma once
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class ContTagTransitionPluginOptions: public PluginOptions
{
public:
    ContTagTransitionPluginOptions() = default;

    virtual ~ContTagTransitionPluginOptions() = default;

    std::string mapPath = "";

    int sizeX = 0;

    int sizeY = 0;

    unsigned int particlePlotLimit = 0;    

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addVDPTagTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addVDPTagTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("map", "mapPath", &ContTagTransitionPluginOptions::mapPath);
        parser->addOption<int>("map", "sizeX", &ContTagTransitionPluginOptions::sizeX);
        parser->addOption<int>("map", "sizeY", &ContTagTransitionPluginOptions::sizeY);
        parser->addOption<unsigned int>("simulation", "particlePlotLimit", &ContTagTransitionPluginOptions::particlePlotLimit);
    }

};
}
