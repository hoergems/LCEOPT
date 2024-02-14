ADD_TERMINAL_PLUGIN(sensorPlacementTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementTerminalPlugin.cpp)
target_include_directories(sensorPlacementTerminalPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/SensorPlacement/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
target_compile_features(sensorPlacementTerminalPlugin PUBLIC cxx_std_17)
