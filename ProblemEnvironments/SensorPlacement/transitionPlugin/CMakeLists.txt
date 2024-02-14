find_package(pinocchio REQUIRED)
#message(FATAL_ERROR ${PINOCCHIO_LIBRARIES})		
ADD_TRANSITION_PLUGIN(sensorPlacementTransitionPluginKin ${CMAKE_CURRENT_LIST_DIR}/SensorPlacementTransitionPluginKin.cpp ${pinocchio_LIBRARIES})
target_include_directories(sensorPlacementTransitionPluginKin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/SensorPlacement/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>
		pinocchio)
target_compile_features(sensorPlacementTransitionPluginKin PUBLIC cxx_std_17)
#target_link_libraries(sensorPlacementTransitionPluginKin PUBLIC pinocchio)
