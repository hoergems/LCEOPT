cmake_minimum_required (VERSION 3.5.1)
project (MPC)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -mfpmath=sse -DEIGEN_NO_DEBUG")

include(GNUInstallDirs)
include(${CMAKE_CURRENT_LIST_DIR}/thirdparty/oppt/src/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/thirdparty/OPPTTree/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/thirdparty/ParticleFilters/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/ProblemEnvironments/CMakeLists.txt)

add_definitions(-DLAZY_SAMPLING)

# Turn off warnings for now
add_definitions(-w)

set(LCEOPT_SRC 
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/LCEOPT.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ParameterVector.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ParameterEvaluator.cpp    
    ${CMAKE_CURRENT_LIST_DIR}/src/ActionNode.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ObservationEdge.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Utils.cpp)
        

add_library( oppt SHARED ${OPPT_SRC} )
target_include_directories(oppt PUBLIC 
     ${oppt_INCLUDE_TARGETS}
     $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/thirdparty/oppt/src>
     $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/thirdparty/FCL/include/>)
target_link_libraries (oppt ${THIRD_PARTY_LIBS} ccd)
set_target_properties(oppt PROPERTIES CXX_STANDARD ${REQUIRED_CXX_STANDARD} CXX_STANDARD_REQUIRED YES)
install(TARGETS oppt
        EXPORT "opptTargets" 
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
        
install(EXPORT opptTargets
  FILE "opptTargets.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt"
)


add_executable(lceopt               
               ${LCEOPT_SRC}
               ${OPPT_TREE_SRC}
               ${PARTICLE_FILTER_SRC})
               
target_include_directories(lceopt
	PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>		
		${CMAKE_CURRENT_LIST_DIR}/thirdparty/OPPTTree/include/
		${CMAKE_CURRENT_LIST_DIR}/thirdparty/ParticleFilters/include
		)
		
target_link_libraries(lceopt PUBLIC 
                      oppt)                      
target_compile_features(lceopt PUBLIC cxx_std_17)
set_target_properties(lceopt
    PROPERTIES    
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin"
)

install(FILES ${CMAKE_CURRENT_LIST_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/ProblemEnvironments/models DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/)
