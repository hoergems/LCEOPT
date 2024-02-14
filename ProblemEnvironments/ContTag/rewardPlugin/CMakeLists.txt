ADD_REWARD_PLUGIN(contTagRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/ContTagRewardPlugin.cpp)

target_include_directories(contTagRewardPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/ContTag/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(contTagRewardPlugin PUBLIC cxx_std_17)
