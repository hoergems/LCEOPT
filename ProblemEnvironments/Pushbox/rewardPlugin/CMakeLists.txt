ADD_REWARD_PLUGIN(pushboxRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/PushboxRewardPlugin.cpp)

target_include_directories(pushboxRewardPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(pushboxRewardPlugin PUBLIC cxx_std_17)
