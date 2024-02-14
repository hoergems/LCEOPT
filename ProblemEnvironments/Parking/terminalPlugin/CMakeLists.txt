ADD_TERMINAL_PLUGIN(parkingTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingTerminalPlugin.cpp)

target_include_directories(parkingTerminalPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Parking/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(parkingTerminalPlugin PUBLIC cxx_std_17)
