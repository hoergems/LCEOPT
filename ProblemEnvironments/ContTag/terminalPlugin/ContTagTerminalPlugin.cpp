#include "oppt/plugin/Plugin.hpp"
#include "TagState.hpp"

namespace oppt
{
class ContTagTerminalPlugin: public TerminalPlugin
{
public :
    ContTagTerminalPlugin():
        TerminalPlugin() {
    }

    virtual ~ContTagTerminalPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        ValidityReportSharedPtr validityReport(new ValidityReport(propagationResult->nextState));
        validityReport->isValid = true;
        validityReport->satisfiesConstraints = true;
        validityReport->collided = false;
        return validityReport;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {
        return propagationResult->nextState->as<TagState>()->tagged();        
    }

};

OPPT_REGISTER_TERMINAL_PLUGIN(ContTagTerminalPlugin)

}




