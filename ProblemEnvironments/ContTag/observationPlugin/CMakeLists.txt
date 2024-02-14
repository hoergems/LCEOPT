ADD_OBSERVATION_PLUGIN(contTagObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/ContTagObservationPlugin.cpp)

target_include_directories(contTagObservationPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/ContTag/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(contTagObservationPlugin PUBLIC cxx_std_17)
