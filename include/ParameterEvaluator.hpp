#ifndef _PARAMETER_EVALUATOR_HPP_
#define _PARAMETER_EVALUATOR_HPP_
#include <oppt/opptCore/core.hpp>
#include <oppt/robotEnvironment/include/RobotEnvironment.hpp>
#include "TreeElement.hpp"
#include "Types.hpp"
#include "LCEOPTOptions.hpp"
#include "ActionNode.hpp"
#include "ObservationEdge.hpp"

namespace oppt {
typedef std::function<ActionSharedPtr(ParameterVector *, const size_t &)> ConstructActionFn;


class ParameterEvaluator {
public:
	/**
	 * @brief Called for continuous action spaces
	 */
	ParameterEvaluator(RobotEnvironment *robotEnvironment,
	                   ProblemEnvironmentOptions *options,
	                   HeuristicPlugin *heuristicPlugin);

	~ParameterEvaluator() = default;

	_NO_COPY_BUT_MOVE(ParameterEvaluator)

	FloatType evaluateParameterVector(ParameterVector *parameterVector, const VectorRobotStatePtr &beliefParticles);

	FloatType evaluateParameterVector(ParameterVector &parameterVector, const VectorRobotStatePtr &beliefParticles);

protected:
	RobotEnvironment *robotEnvironment_ = nullptr;

	ProblemEnvironmentOptions *options_ = nullptr;

	HeuristicPlugin *heuristicPlugin_ = nullptr;

	PropagationRequestSharedPtr propReq_ = nullptr;

	PropagationResultSharedPtr propRes_ = nullptr;

	ObservationRequestSharedPtr obsReq_ = nullptr;

	HeuristicInfo heuristicInfo;

	TreeElementPtr root_ = nullptr;

	unsigned int numActionDimensions_ = 0;

	ConstructActionFn constructActionFn_;

	// For discrete actions
	std::vector<ActionSharedPtr> allActions_;

protected:
	void buildPolicyTree_(TreeElement *treeElement, int &depth);

	RobotStateSharedPtr sampleStateFromBelief(const VectorRobotStatePtr &beliefParticles) const;	

	FloatType search(RobotStateSharedPtr &state,
	                 TreeElement *actionNode,
	                 ParameterVector *parameterVector,
	                 const int &currentDepth);

};

}

#endif