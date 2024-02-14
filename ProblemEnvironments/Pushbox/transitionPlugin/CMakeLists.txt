ADD_TRANSITION_PLUGIN(pushbox2DTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/Pushbox2DTransitionPlugin.cpp)
ADD_TRANSITION_PLUGIN(pushbox3DTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/Pushbox3DTransitionPlugin.cpp)

target_include_directories(pushbox2DTransitionPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_include_directories(pushbox3DTransitionPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(pushbox2DTransitionPlugin PUBLIC cxx_std_17)
target_compile_features(pushbox3DTransitionPlugin PUBLIC cxx_std_17)
