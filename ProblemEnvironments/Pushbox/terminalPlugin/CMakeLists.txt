ADD_TERMINAL_PLUGIN(pushboxTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/PushboxTerminalPlugin.cpp)

target_include_directories(pushboxTerminalPlugin 
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/Pushbox/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(pushboxTerminalPlugin PUBLIC cxx_std_17)
