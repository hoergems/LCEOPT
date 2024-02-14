ADD_INITIAL_BELIEF_PLUGIN(contTagInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/ContTagInitialBeliefPlugin.cpp)

target_include_directories(contTagInitialBeliefPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/ContTag/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(contTagInitialBeliefPlugin PUBLIC cxx_std_17)
