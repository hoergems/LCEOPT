ADD_INITIAL_BELIEF_PLUGIN(parkingInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingInitialBeliefPlugin.cpp)

target_include_directories(parkingInitialBeliefPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Parking/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(parkingInitialBeliefPlugin PUBLIC cxx_std_17)
