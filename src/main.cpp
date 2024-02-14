#include <oppt/problemEnvironment/ProblemEnvironment.hpp>
#include "LCEOPT.hpp"
#include "LCEOPTOptions.hpp"

int main(int argc, char const* argv[]) {
	oppt::ProblemEnvironment p;
	p.setup<solvers::LCEOPT, oppt::LCEOPTOptions>(argc, argv);
	p.runEnvironment(argc, argv);
	return 0;
}