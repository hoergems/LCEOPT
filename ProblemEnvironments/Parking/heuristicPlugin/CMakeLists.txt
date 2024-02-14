ADD_HEURISTIC_PLUGIN(parkingHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/ParkingHeuristicPlugin.cpp)

target_include_directories(parkingHeuristicPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Parking/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(parkingHeuristicPlugin PUBLIC cxx_std_17)
