ADD_TERMINAL_PLUGIN(contTagTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/ContTagTerminalPlugin.cpp)

target_include_directories(contTagTerminalPlugin
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ProblemEnvironments/ContTag/shared>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/src>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/include/>)
		
target_compile_features(contTagTerminalPlugin PUBLIC cxx_std_17)
