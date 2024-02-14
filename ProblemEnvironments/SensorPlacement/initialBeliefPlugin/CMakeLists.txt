ADD_INITIAL_BELIEF_PLUGIN(sensorPlacementInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementInitialBeliefPlugin.cpp)
target_include_directories(sensorPlacementInitialBeliefPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/SensorPlacement/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)		
target_compile_features(sensorPlacementInitialBeliefPlugin PUBLIC cxx_std_17)
