ADD_OBSERVATION_PLUGIN(sensorPlacementObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementObservationPlugin.cpp)
target_include_directories(sensorPlacementObservationPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/SensorPlacement/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
target_compile_features(sensorPlacementObservationPlugin PUBLIC cxx_std_17)
