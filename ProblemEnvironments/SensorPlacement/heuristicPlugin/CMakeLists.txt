ADD_HEURISTIC_PLUGIN(sensorPlacementHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementHeuristicPlugin.cpp)
target_include_directories(sensorPlacementHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/SensorPlacement/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
target_compile_features(sensorPlacementHeuristicPlugin PUBLIC cxx_std_17)
