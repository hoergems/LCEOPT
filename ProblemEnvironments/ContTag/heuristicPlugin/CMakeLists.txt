ADD_HEURISTIC_PLUGIN(contTagHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/ContTagHeuristicPlugin.cpp)

target_include_directories(contTagHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/ContTag/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(contTagHeuristicPlugin PUBLIC cxx_std_17)
