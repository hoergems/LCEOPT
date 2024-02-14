ADD_INITIAL_BELIEF_PLUGIN(pushbox2DInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/Pushbox2DInitialBeliefPlugin.cpp)
ADD_INITIAL_BELIEF_PLUGIN(pushbox3DInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/Pushbox3DInitialBeliefPlugin.cpp)

target_include_directories(pushbox2DInitialBeliefPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
target_include_directories(pushbox3DInitialBeliefPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
target_compile_features(pushbox2DInitialBeliefPlugin PUBLIC cxx_std_17)
target_compile_features(pushbox3DInitialBeliefPlugin PUBLIC cxx_std_17)
