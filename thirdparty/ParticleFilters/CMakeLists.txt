set(PARTICLE_FILTER_SRC
    ${CMAKE_CURRENT_LIST_DIR}/src/LowVarianceFilter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/BootstrapFilter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RejectionSamplingFilter.cpp)
#target_include_directories(ParticleFilters 
#    PUBLIC 
#        $<INSTALL_INTERFACE:include> 
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    PRIVATE
#        oppt)
#target_link_libraries(ParticleFilters PUBLIC oppt)

#target_compile_features(ParticleFilters PUBLIC cxx_std_14)
#install(TARGETS ParticleFilters
#        EXPORT "particleFiltersTargets"
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
#install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ParticleFilters/
#  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
#  PATTERN ".DS_Store" EXCLUDE
#)
#include(CMakePackageConfigHelpers)
#configure_package_config_file("cmake/particleFiltersConfig.cmake.in"
#  "${CMAKE_CURRENT_BINARY_DIR}/particleFiltersConfig.cmake"
#  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
#  PATH_VARS CMAKE_INSTALL_INCLUDEDIR
#)
#install(EXPORT particleFiltersTargets
#  FILE "particleFiltersTargets.cmake"
#  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
#)

#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/particleFiltersConfig.cmake"
#  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
#)
