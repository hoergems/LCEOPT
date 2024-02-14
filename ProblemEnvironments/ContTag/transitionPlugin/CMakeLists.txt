ADD_TRANSITION_PLUGIN(contTagTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/ContTagTransitionPlugin.cpp)

target_include_directories(contTagTransitionPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/ContTag/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(contTagTransitionPlugin PUBLIC cxx_std_17)
