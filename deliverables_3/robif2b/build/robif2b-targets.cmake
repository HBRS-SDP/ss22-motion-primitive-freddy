# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget robif2b::2dof robif2b::ethercat robif2b::kelo)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target robif2b::2dof
add_library(robif2b::2dof SHARED IMPORTED)

set_target_properties(robif2b::2dof PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/barath/sdp/robif2b/include"
)

# Create imported target robif2b::ethercat
add_library(robif2b::ethercat SHARED IMPORTED)

set_target_properties(robif2b::ethercat PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/barath/sdp/robif2b/include"
)

# Create imported target robif2b::kelo
add_library(robif2b::kelo SHARED IMPORTED)

set_target_properties(robif2b::kelo PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/barath/sdp/robif2b/include"
)

# Import target "robif2b::2dof" for configuration ""
set_property(TARGET robif2b::2dof APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robif2b::2dof PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/barath/sdp/robif2b/build/src/nbx/2dof/lib2dof.so"
  IMPORTED_SONAME_NOCONFIG "lib2dof.so"
  )

# Import target "robif2b::ethercat" for configuration ""
set_property(TARGET robif2b::ethercat APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robif2b::ethercat PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "soem"
  IMPORTED_LOCATION_NOCONFIG "/home/barath/sdp/robif2b/build/src/nbx/ethercat/libethercat.so"
  IMPORTED_SONAME_NOCONFIG "libethercat.so"
  )

# Import target "robif2b::kelo" for configuration ""
set_property(TARGET robif2b::kelo APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robif2b::kelo PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/barath/sdp/robif2b/build/src/nbx/kelo/libkelo.so"
  IMPORTED_SONAME_NOCONFIG "libkelo.so"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
