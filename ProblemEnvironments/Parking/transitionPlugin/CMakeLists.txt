ADD_TRANSITION_PLUGIN(parkingTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingTransitionPlugin.cpp)

target_include_directories(parkingTransitionPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Parking/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(parkingTransitionPlugin PUBLIC cxx_std_17)
