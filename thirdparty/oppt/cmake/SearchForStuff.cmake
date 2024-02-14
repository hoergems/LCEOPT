SET(BOOST_MIN_VERSION "1.55.0")

include(FindPkgConfig)
include(GNUInstallDirs)

set(oppt_LIBRARY_DIRS "")
set(oppt_INCLUDE_TARGETS "")

#############################
if (GAZEBO_SUPPORT)
    message(STATUS "Building with Gazebo support")
    include(${CMAKE_CURRENT_LIST_DIR}/FindGazebo.cmake)
    set(GZ_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS})
    list(APPEND oppt_INCLUDE_TARGETS "${GAZEBO_INCLUDE_DIRS}") 
endif()


#############################
include(${CMAKE_CURRENT_LIST_DIR}/FindTinyXML.cmake)
if (NOT TINYXML_FOUND)
    message(FATAL_ERROR "TinyXML could not be found")
endif()
list(APPEND oppt_INCLUDE_TARGETS "${TINYXML_INCLUDE_DIRS}")

#############################
set (SDFormat_VERSION 4.1.0)
find_package(sdformat12 ${SDFormat_MIN_VERSION} REQUIRED)
if(NOT sdformat12_FOUND)   
   message(FATAL_ERROR "SDF could not be found")
endif()
list(APPEND oppt_INCLUDE_TARGETS "${SDFormat_INCLUDE_DIRS}")

#############################
find_package(Eigen3 REQUIRED)
if (EIGEN3_VERSION_STRING LESS 3.3.4)    
else()    
    add_definitions(-DEIGEN_GT_3_3_4)
endif()
list(APPEND oppt_INCLUDE_TARGETS "${EIGEN3_INCLUDE_DIRS}")


#############################
include(${CMAKE_CURRENT_LIST_DIR}/FindAssimp.cmake)
#find_package(ASSIMP REQUIRED)
if (NOT ASSIMP_FOUND)
   message(FATAL_ERROR "Assimp not found")
endif()
list(APPEND oppt_INCLUDE_TARGETS "${ASSIMP_INCLUDE_PATH}") 

#############################
find_package(ccd REQUIRED)
if (GAZEBO_SUPPORT)
	if (GAZEBO_VERSION LESS 11)
	  if(PKG_CONFIG_FOUND)
	    pkg_check_modules(FCL fcl)    
	    if(NOT FCL_FOUND)
	       message(FATAL_ERROR "FCL could not be found")
	    endif()
	    if (FCL_VERSION GREATER 0.4.0)
	        set(FCL_GT_0_4 True)
		add_definitions(-DFCL_GT_0_4)
	    endif()       
	  endif()
	  list(APPEND oppt_INCLUDE_TARGETS "${FCL_INCLUDE_DIRS}")
	else()
	  add_definitions(-DFCL_GT_0_4)    
	endif()
endif()
   

#############################
set(USE_RVIZ False)
if (BUILD_VIEWER)
    find_package(catkin COMPONENTS roscpp rviz QUIET)
    if (catkin_FOUND)
        message(STATUS "ROS and Rviz have been found compiling with viewer support")
        set(USE_RVIZ True)
        add_definitions(-DUSE_RVIZ)
        list(APPEND oppt_INCLUDE_TARGETS "${roscpp_INCLUDE_DIRS}")
    else()
        message(STATUS "ROS and Rviz couldn't be found. Compiling without viewer support")   
    endif()
endif()

#############################
set(SUPPORTS_IK True)

find_package(kdl_parser QUIET)
find_package(trac_ik_lib QUIET)
if(NOT kdl_parser_FOUND)
    message(STATUS "kdl_parser could not be found. Building without IK support")
    set(SUPPORTS_IK False)
endif()
if(NOT trac_ik_lib_FOUND)
    message(STATUS "trac_ik_lib could not be found. Building without IK support")
    set(SUPPORTS_IK False)
endif()
if (SUPPORTS_IK)  
    message(STATUS "Compiling with IK support")
    add_definitions(-DSUPPORTS_IK)
    list(APPEND oppt_INCLUDE_TARGETS "${kdl_parser_INCLUDE_DIRS}")
    list(APPEND oppt_INCLUDE_TARGETS "${trac_ik_lib_INCLUDE_DIRS}")
endif()


#############################
find_package(Boost
             REQUIRED 
             system 
             thread             
             filesystem 
             serialization)
if (Boost_FOUND)    
    list(APPEND oppt_INCLUDE_TARGETS "${Boost_INCLUDE_DIRS}") 
endif ()
