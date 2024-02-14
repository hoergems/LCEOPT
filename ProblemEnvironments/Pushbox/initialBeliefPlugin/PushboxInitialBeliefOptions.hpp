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
class PushboxInitialBeliefOptions: public PluginOptions
{
public:
    PushboxInitialBeliefOptions() = default;

    virtual ~PushboxInitialBeliefOptions() = default;

    VectorFloat initialStateVec;

    FloatType initialBoxPositionUncertainty = 0.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addPushboxInitialBeliefOptions(parser.get());
        return std::move(parser);
    }

    static void addPushboxInitialBeliefOptions(options::OptionParser* parser) {
        parser->addOption<VectorFloat>("initialBeliefOptions", "initialStateVec", &PushboxInitialBeliefOptions::initialStateVec);
        parser->addOption<FloatType>("initialBeliefOptions",
                                     "initialBoxPositionUncertainty",
                                     &PushboxInitialBeliefOptions::initialBoxPositionUncertainty);
    }

};
}

#endif
