project(ss22_project)
cmake_minimum_required(VERSION 3.0.2)

find_package(robif2b REQUIRED)
find_package(kelo_sdp REQUIRED)

include(GNUInstallDirs)

set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})



add_executable(us1 
  src/kelo_robile_example_us1.c
)
target_link_libraries(us1
  robif2b::kelo
  robif2b::ethercat
  kelo_sdp::robile_kinematics
  m
)


add_executable(us2 
  src/kelo_robile_example_us2.c
)
target_link_libraries(us2
  robif2b::kelo
  robif2b::ethercat
  kelo_sdp::robile_kinematics
  m
)

add_executable(us3 
  src/kelo_robile_example_us3.c
)
target_link_libraries(us3
  robif2b::kelo
  robif2b::ethercat
  m
)

add_executable(us5
  src/kelo_robile_example_us5.c
)
target_link_libraries(us5
  robif2b::kelo
  robif2b::ethercat
  m
)
target_include_directories(us5
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)



add_executable(main
  src/main.c
  src/wheel_aligement.c
  src/base_aligement.c
)
target_link_libraries(main
  robif2b::kelo
  robif2b::ethercat
  m
  
)
target_include_directories(main
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
  TARGETS us2 us3 us5 main
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Export this package to CMake's package registry
if(ENABLE_PACKAGE_REGISTRY)
  export(PACKAGE ${PROJECT_NAME})
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DIR}
)

# Generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  VERSION "0.1.0"
  COMPATIBILITY AnyNewerVersion
)

# Install cmake configuration and package version
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION ${CMAKE_INSTALL_DIR}
)

# Make the targets accessible from this packages's build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR}
)
