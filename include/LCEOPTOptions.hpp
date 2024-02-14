#ifndef _LCEOPT_OPTIONS_HPP_
#define _LCEOPT_OPTIONS_HPP_
#include <oppt/problemEnvironment/ProblemEnvironmentOptions.hpp>

namespace oppt
{
struct LCEOPTOptions: public oppt::ProblemEnvironmentOptions {
public:
    LCEOPTOptions() = default;
    virtual ~LCEOPTOptions() = default;

    std::string particleFilter = "";

    unsigned int minParticleCount = 0;

    unsigned int nEffectiveParticles = 0;

    unsigned int numParameterVectors = 0;

    FloatType numEliteSamplesFactor = 0;

    unsigned int maxNumIterations = 0;

    unsigned int numTrajectoriesPerParameterVector = 1;

    unsigned int numObservationEdges = 0;

    unsigned int policyTreeDepth = 0;

    FloatType maxCovarianceSum = 0.0;

    FloatType initialStdDev = 0.0;

    FloatType learningRate = 0.0;

    bool debug = false;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            ProblemEnvironmentOptions::makeParser(simulating);
        addMPCOptions(parser.get());
        return std::move(parser);
    }

    static void addMPCOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("LCEOPT", "particleFilter", &LCEOPTOptions::particleFilter);
        parser->addOption<unsigned int>("LCEOPT", "minParticleCount", &LCEOPTOptions::minParticleCount);
        parser->addOption<unsigned int>("LCEOPT", "nEffectiveParticles", &LCEOPTOptions::nEffectiveParticles);
        parser->addOption<unsigned int>("LCEOPT", "numParameterVectors", &LCEOPTOptions::numParameterVectors);
        parser->addOption<FloatType>("LCEOPT", "numEliteSamplesFactor", &LCEOPTOptions::numEliteSamplesFactor);
        parser->addOption<unsigned int>("LCEOPT", "maxNumIterations", &LCEOPTOptions::maxNumIterations);
        parser->addOption<unsigned int>("LCEOPT",
                                        "numTrajectoriesPerParameterVector",
                                        &LCEOPTOptions::numTrajectoriesPerParameterVector);
        parser->addOptionWithDefault<FloatType>("LCEOPT", "maxCovarianceSum", &LCEOPTOptions::maxCovarianceSum, 0.0);
        parser->addOption<unsigned int>("LCEOPT", "numObservationEdges", &LCEOPTOptions::numObservationEdges);
        parser->addOption<unsigned int>("LCEOPT", "policyTreeDepth", &LCEOPTOptions::policyTreeDepth);
        parser->addOption<FloatType>("LCEOPT", "initialStdDev", &LCEOPTOptions::initialStdDev);
        parser->addOptionWithDefault<FloatType>("LCEOPT", "learningRate", &LCEOPTOptions::learningRate, 1.0);
        parser->addOptionWithDefault<bool>("LCEOPT", "debug", &LCEOPTOptions::debug, false);

    }
};
}

#endif
