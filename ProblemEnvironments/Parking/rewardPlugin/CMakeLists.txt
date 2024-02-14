ADD_REWARD_PLUGIN(parkingRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingRewardPlugin.cpp)

target_include_directories(parkingRewardPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Parking/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(parkingRewardPlugin PUBLIC cxx_std_17)
