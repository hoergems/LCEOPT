#pragma once
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class ContTagInitialBeliefOptions: public PluginOptions
{
public:
    ContTagInitialBeliefOptions() = default;

    virtual ~ContTagInitialBeliefOptions() = default;

    std::string mapPath = "";

    int sizeX = 0;

    int sizeY = 0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addVDPTagTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addVDPTagTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("map", "mapPath", &ContTagInitialBeliefOptions::mapPath);
        parser->addOption<int>("map", "sizeX", &ContTagInitialBeliefOptions::sizeX);
        parser->addOption<int>("map", "sizeY", &ContTagInitialBeliefOptions::sizeY);

    }

};
}
