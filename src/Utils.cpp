#include "Utils.hpp"
#include <iomanip>

namespace oppt {
void ToTree(const std::vector<Matrixdf> probabilityMatrices, const int &numObservationEdges, const int maxDepth) {
	int numNodes = probabilityMatrices.size();

	auto getNodeDepth = [&maxDepth, &numObservationEdges](const int &node) {
		int depth = 0;
		int parentIdx = 0;
		if (node != 0) {
			depth++;
			parentIdx = std::floor((node - 1) / numObservationEdges);
		}

		while (parentIdx != 0) {
			depth++;
			parentIdx = std::floor((parentIdx - 1) / numObservationEdges);
		}

		return depth;
	};

	std::string output = "digraph g{\n";
	for (size_t i = 0; i != numNodes; ++i) {
		int depth = getNodeDepth(i);
		if (depth == maxDepth)
			break;
		output += "n" + std::to_string(i);
		output += "[label=\"";
		for (size_t row = 0; row != probabilityMatrices[i].rows(); ++row) {
			output += "[";
			for (size_t col = 0; col != probabilityMatrices[i].cols(); ++col) {
				std::stringstream ss;
				ss << std::fixed << std::setprecision(2) << probabilityMatrices[i](row, col);
				output += ss.str();
				if (col != probabilityMatrices[i].cols() - 1)
					output += " ";
			}

			output += "]";

			if (row != probabilityMatrices[i].rows() - 1) {
				output += "\\";
				output += "n ";
			}
		}

		output += "\"]\n";
	}

	output += "\n";
	for (size_t node = 1; node != numNodes; ++node) {
		if (getNodeDepth(node) == maxDepth)
			break;
		int parentNode = (int)(floor((node - 1) / numObservationEdges));
		output += "n" + std::to_string(parentNode) + " -> n" + std::to_string(node) + ";\n";
	}

	output += "}";

	std::ofstream out("tree.dot");
	out << output;
	out.close();
}

void ToTree(const Matrixdf &covariance, const int &numObservationEdges, const int maxDepth) {
	int numNodes = covariance.rows();
	auto getNodeDepth = [&maxDepth, &numObservationEdges](const int &node) {
		int depth = 0;
		int parentIdx = 0;
		if (node != 0) {
			depth++;
			parentIdx = std::floor((node - 1) / numObservationEdges);
		}

		while (parentIdx != 0) {
			depth++;
			parentIdx = std::floor((parentIdx - 1) / numObservationEdges);
		}

		return depth;
	};

	std::string output = "digraph g{\n";
	for (size_t i = 0; i != numNodes; ++i) {
		int depth = getNodeDepth(i);
		if (depth == maxDepth)
			break;
		output += "n" + std::to_string(i);
		output += "[label=\"[";
		for (size_t col = 0; col != covariance.cols(); ++col) {
			std::stringstream ss;
			ss << std::fixed << std::setprecision(2) << covariance(i, col);
			output += ss.str();
			if (col != covariance.cols() - 1)
				output += " ";					
		}

		output += "]";	

		/**for (size_t row = 0; row != covariance.rows(); ++row) {
			output += "[";
			for (size_t col = 0; col != covariance.cols(); ++col) {
				std::stringstream ss;
				ss << std::fixed << std::setprecision(2) << covariance(row, col);
				output += ss.str();
				if (col != covariance.cols() - 1)
					output += " ";
			}

			output += "]";
			cout << "output" << endl << output << endl;
			getchar();

			if (row != covariance.rows() - 1) {
				output += "\\";
				output += "n ";
			}
		}*/

		output += "\"]\n";
	}

	output += "\n";
	for (size_t node = 1; node != numNodes; ++node) {
		if (getNodeDepth(node) == maxDepth)
			break;
		int parentNode = (int)(floor((node - 1) / numObservationEdges));
		output += "n" + std::to_string(parentNode) + " -> n" + std::to_string(node) + ";\n";
	}

	output += "}";

	std::ofstream out("tree.dot");
	out << output;
	out.close();
}



}