/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_OPTIONS_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class PushboxTransitionPluginOptions: public PluginOptions
{
public:
    PushboxTransitionPluginOptions() = default;

    virtual ~PushboxTransitionPluginOptions() = default;

    FloatType boxSpeedUncertainty = 0.0;

    FloatType boxPositionMoveUncertainty = 0.0;

    VectorFloat goalPosition;

    FloatType goalRadius = 0.0;

    unsigned int particlePlotLimit = 0;

    std::string goalLink = "";
    size_t sizeX = 0;
    size_t sizeY = 0;

    std::string robotName = "";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPushboxTransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addPushboxTransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "boxSpeedUncertainty",
                                     &PushboxTransitionPluginOptions::boxSpeedUncertainty);
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "boxPositionMoveUncertainty",
                                     &PushboxTransitionPluginOptions::boxPositionMoveUncertainty);
        parser->addOption<size_t>("map", "sizeX", &PushboxTransitionPluginOptions::sizeX);
        parser->addOption<size_t>("map", "sizeY", &PushboxTransitionPluginOptions::sizeY);
        parser->addOption<VectorFloat>("map", "goalPosition", &PushboxTransitionPluginOptions::goalPosition);
        parser->addOption<FloatType>("map", "goalRadius", &PushboxTransitionPluginOptions::goalRadius);
        parser->addOption<unsigned int>("simulation", "particlePlotLimit", &PushboxTransitionPluginOptions::particlePlotLimit);
        parser->addOption<std::string>("problem", "robotName", &PushboxTransitionPluginOptions::robotName);

    }

};
}

#endif
