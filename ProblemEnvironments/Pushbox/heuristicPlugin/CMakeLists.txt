ADD_HEURISTIC_PLUGIN(pushboxHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/PushboxHeuristicPlugin.cpp)

target_include_directories(pushboxHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(pushboxHeuristicPlugin PUBLIC cxx_std_17)
