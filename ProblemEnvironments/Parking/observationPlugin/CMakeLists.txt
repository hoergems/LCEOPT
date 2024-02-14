ADD_OBSERVATION_PLUGIN(parkingObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingObservationPlugin.cpp)

target_include_directories(parkingObservationPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Parking/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(parkingObservationPlugin PUBLIC cxx_std_17)
