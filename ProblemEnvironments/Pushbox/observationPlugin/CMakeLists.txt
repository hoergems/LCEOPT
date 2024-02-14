ADD_OBSERVATION_PLUGIN(pushbox2DObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/Pushbox2DObservationPlugin.cpp)
ADD_OBSERVATION_PLUGIN(pushbox3DObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/Pushbox3DObservationPlugin.cpp)

target_include_directories(pushbox2DObservationPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_include_directories(pushbox3DObservationPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(pushbox2DObservationPlugin PUBLIC cxx_std_17)
target_compile_features(pushbox3DObservationPlugin PUBLIC cxx_std_17)

