option(BUILD_VIEWER "Build the viewer" OFF)
option(GAZEBO_SUPPORT "Build with Gazebo support" OFF)

add_definitions(-DUSE_DOUBLE_PRECISION=true)
if (GAZEBO_SUPPORT)
    add_definitions(-DGZ_SUPPORT)
endif()

include(FindPkgConfig)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(OPPT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/oppt/")
set (OPPT_CMAKE_DIR ${OPPT_PATH}/cmake)
include (${OPPT_CMAKE_DIR}/SearchForStuff.cmake)

if(GZ_GT_10)
  set(REQUIRED_CXX_STANDARD 17)
else()
  set(REQUIRED_CXX_STANDARD 11)
endif()

configure_package_config_file("${OPPT_CMAKE_DIR}/opptMacros.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/opptMacros.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt"
)

include(${CMAKE_CURRENT_BINARY_DIR}/opptMacros.cmake)
include(${OPPT_PATH}/src/oppt/options/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/opptCore/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/plugin/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/robotEnvironment/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/robots/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/utils/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/filter/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/problemEnvironment/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/robotHeaders/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/solver/CMakeLists.txt)
include(${OPPT_PATH}/src/oppt/CMakeLists.txt)
if (GAZEBO_SUPPORT)    
    include(${OPPT_PATH}/src/oppt/gazeboInterface/CMakeLists.txt)
endif()

if (USE_RVIZ AND BUILD_VIEWER)
  include(${OPPT_PATH}/src/oppt/viewerPublisher/CMakeLists.txt)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FCL/src/CMakeLists.txt)

set(COMMON_SRCS
    ${THIRD_PARTY_SRC}
    ${CMAKE_CURRENT_LIST_DIR}/oppt/global.cpp)
    
set(OPPT_SRC
    ${COMMON_SRCS}
    ${OPTIONS_SRC}
    ${ROBOT_PROBLEM_SRC}    
    ${ROBOT_ENVIRONMENT_SRC}    
    ${GAZEBO_INTERFACE_SRC}    
    ${ROBOTS_SRC}    
    ${OPPT_UTILS_SRC}
    ${FILTER_SRC}
    ${VIEWER_PUBLISHER_SRC}    
    ${CHANGES_PARSER_SRC}
    ${FCL_SRC})
    
set(THIRD_PARTY_LIBS
    ${Boost_LIBRARIES}
    ${FCL_LIBRARIES}                         
    ${TCLAP_LIBRARIES}                        
    ${LIBSPATIALINDEX_LIBRARIES}    
    ${catkin_LIBRARIES}
    ${SDFormat_LIBRARIES}
    ${GAZEBO_LIBRARIES}          
    ${URDFDOM_LIBRARIES}    
    ${ASSIMP_LIBRARIES}
    ${TINYXML_LIBRARIES}    
    ${THIRD_PARTY_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${kdl_parser_LIBRARIES} 
    ${trac_ik_lib_LIBRARIES}    
    ${CMAKE_DL_LIBS})
