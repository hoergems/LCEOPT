
# Lazy Cross-Entropy Search over Policy Trees (LCEOPT)
This repository contains the source code for the paper

**A Surprisingly Simple Continuous-Action POMDP Solver: Lazy Cross-Entropy Search Over Policy Trees** [[Paper]](https://arxiv.org/abs/2305.08049)  

by
[Marcus Hoerger](mailto:m.hoerger@uq.edu.au), [Hanna Kurniawati](mailto:hanna.kurniawati@anu.edu.au), [Dirk Kroese](mailto:kroese@maths.uq.edu.au) and [Nan Ye](mailto:nan.ye@uq.edu.au).

## Citation
If you use this repository in your research, please cite the paper as follows:
```
@inproceedings{hoerger2024LCEOPT,
  title={A Surprisingly Simple Continuous-Action {POMDP} Solver: Lazy Cross-Entropy Search Over Policy Trees},
  author={Marcus Hoerger and Hanna Kurniawati and Dirk Kroese and Nan Ye},
  booktitle={Proceedings of the AAAI Conference on Artificial Intelligence},
  year={2024},
  url={https://arxiv.org/abs/2305.08049}
}
```
## Directories
| Directory/File | Description |
| ------ | ------ |
| ProblemEnvironments | Contains source code for the POMDP problems used in the paper |
| include | Contains the header files of LCEOPT |
| src | Contains the source code of LCEOPT |
| thirdparty | Contains the source code of the third-party libraries used by LCEOPT |

## Requirements
- Ubuntu 22.04 or higher

## Installation
First, clone the repository and install the dependencies:
		
	git clone https://github.com/LCEOPT.git <LCEOPT_DIRECTORY>
	cd <LCEOPT_DIRECTORY> && ./install_dependencies.sh

where ``<LCEOPT_DIRECTORY>`` is a directory of your choice. 
Once completed, build LCEOPT via

	mkdir -p build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIRECTORY> ..
	make -j <NUMBER_OF_BUILD_JOBS> && make install

where ``<INSTALL_DIRECTORY>`` is a directory of your choice.

## Quick Start
After successfully building and installing LCEOPT, open a terminal and run

	source <INSTALL_DIRECTORY>/share/oppt/setup.sh

This must be executed in each new terminal. Alternatively, add this line to your ~/.bashrc file.
In the same terminal, run

	<LCEOPT_DIRECTORY>/bin/lceopt --cfg <LCEOPT_DIRECTORY>/ProblemEnvironments/<PROBLEM>/cfg/<PROBLEM_CONFIG_FILE>.cfg

For instance, to run the ContTag problem, run

	<LCEOPT_DIRECTORY>/bin/lceopt --cfg <LCEOPT_DIRECTORY>/ProblemEnvironments/ContTag/cfg/ContTag.cfg

## Acknowledgements
Our implementation of LCEOPT uses the following third-party libraries:
- [OPPT](https://github.com/RDLLab/oppt)
- [FCL](https://github.com/flexible-collision-library/fcl)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
