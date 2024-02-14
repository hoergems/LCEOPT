#include "ParameterEvaluator.hpp"

namespace oppt {

ParameterEvaluator::ParameterEvaluator(RobotEnvironment *robotEnvironment,
                                       ProblemEnvironmentOptions *options,
                                       HeuristicPlugin *heuristicPlugin):
	robotEnvironment_(robotEnvironment),
	options_(options),
	heuristicPlugin_(heuristicPlugin),
	numActionDimensions_(robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions()) {
	root_ = TreeElementPtr(new ActionNode(nullptr));
	root_->as<ActionNode>()->setIdx(0);
	int depth = 0;
	buildPolicyTree_(root_.get(), depth);
	propReq_ = PropagationRequestSharedPtr(new PropagationRequest);
	propRes_ = PropagationResultSharedPtr(new PropagationResult);
	obsReq_ = ObservationRequestSharedPtr(new ObservationRequest);

	constructActionFn_ = [this](ParameterVector * parameterVector, const size_t &actionNodeIdx) {
		VectorFloat actionVec(numActionDimensions_, 0.0);
		for (size_t i = 0; i != numActionDimensions_; ++i) {
			actionVec[i] = parameterVector->operator()(actionNodeIdx * numActionDimensions_ + i);
		}

		return ActionSharedPtr(new VectorAction(actionVec));
	};

}

FloatType ParameterEvaluator::evaluateParameterVector(ParameterVector *parameterVector, const VectorRobotStatePtr &beliefParticles) {
	auto options = static_cast<const LCEOPTOptions *>(options_);
	FloatType avgReward = 0.0;

	// Sample trajectories to evaluate parameter vector
	for (size_t i = 0; i != options->numTrajectoriesPerParameterVector; ++i) {
		TreeElement *actionNode = root_.get();
		// Sample state from the belief current belief
		RobotStateSharedPtr state = sampleStateFromBelief(beliefParticles);

		// Evaluate parameter vector given the sampled states		
		FloatType totalDiscReward = search(state, actionNode, parameterVector, 0);
		avgReward += (totalDiscReward - avgReward) / (i + 1);
	}

	return avgReward;
}

FloatType ParameterEvaluator::evaluateParameterVector(ParameterVector &parameterVector, const VectorRobotStatePtr &beliefParticles) {
	return evaluateParameterVector(&parameterVector, beliefParticles);
}


RobotStateSharedPtr ParameterEvaluator::sampleStateFromBelief(const VectorRobotStatePtr &beliefParticles) const {
	std::uniform_int_distribution<unsigned int> distr(0, beliefParticles.size() - 1);
	return beliefParticles[distr(*(robotEnvironment_->getRobot()->getRandomEngine().get()))];
}

void ParameterEvaluator::buildPolicyTree_(TreeElement *treeElement, int &depth) {
	auto options = static_cast<const LCEOPTOptions *>(options_);
	for (size_t i = 0; i != options->numObservationEdges; ++i) {
		TreeElementPtr observationEdge(new ObservationEdge(treeElement));
		TreeElement *observationEdgePtr = observationEdge.get();
		treeElement->addChild(std::move(observationEdge));

		TreeElementPtr actionNode(new ActionNode(observationEdgePtr));
		actionNode->as<ActionNode>()->setIdx(treeElement->as<ActionNode>()->getIdx() * options->numObservationEdges + i + 1);
		TreeElement *actionNodePtr = actionNode.get();
		observationEdgePtr->addChild(std::move(actionNode));
		int d = depth + 1;
		if (d != options->policyTreeDepth)
			buildPolicyTree_(actionNodePtr, d);
	}
}

FloatType ParameterEvaluator::search(RobotStateSharedPtr &state,
                                     TreeElement *actionNode,
                                     ParameterVector *parameterVector,
                                     const int &currentDepth) {
	auto options = static_cast<const LCEOPTOptions *>(options_);

	if (currentDepth > options->policyTreeDepth) {
		// Get heuristic
		heuristicInfo.currentState = state;
		heuristicInfo.currentStep = currentDepth;
		heuristicInfo.discountFactor = options_->discountFactor;
		FloatType heuristicValue = heuristicPlugin_->getHeuristicValue(&heuristicInfo);
		return heuristicValue;
	}

	// Get the action associated to the action node. If there is no associated action, sample one	
	auto actionNodeIdx = actionNode->as<ActionNode>()->getIdx();
	ActionSharedPtr action = constructActionFn_(parameterVector, actionNodeIdx);
	propReq_->currentState = state;
	propReq_->action = action.get();

	// Sample next state
	propRes_ = robotEnvironment_->getRobot()->propagateState(propReq_);

	// Get immediate reward
	FloatType reward = robotEnvironment_->getReward(propRes_);
	if (robotEnvironment_->isTerminal(propRes_)) 
		return reward;	

	state = propRes_->nextState;
	int depth = currentDepth + 1;
	if (depth != options->policyTreeDepth + 1) {
		// Sample observation
		obsReq_->currentState = state;
		obsReq_->action = action.get();
		auto observation = robotEnvironment_->getRobot()->makeObservationReport(obsReq_)->observation;
		TreeElement *observationEdge = (*(actionNode->getChildren() + observation->as<DiscreteVectorObservation>()->getBinNumber())).get();

		// Go to next action node
		actionNode = (*(observationEdge->getChildren())).get();
	}

	return reward + options_->discountFactor * search(state, actionNode, parameterVector, depth);
}
}