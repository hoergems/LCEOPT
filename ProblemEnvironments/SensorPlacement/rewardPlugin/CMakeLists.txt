ADD_REWARD_PLUGIN(sensorPlacementRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementRewardPlugin.cpp)
target_include_directories(sensorPlacementRewardPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/SensorPlacement/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
target_compile_features(sensorPlacementRewardPlugin PUBLIC cxx_std_17)
