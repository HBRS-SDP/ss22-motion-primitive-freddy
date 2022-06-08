#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robif2b::2dof" for configuration ""
set_property(TARGET robif2b::2dof APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robif2b::2dof PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/lib2dof.so"
  IMPORTED_SONAME_NOCONFIG "lib2dof.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robif2b::2dof )
list(APPEND _IMPORT_CHECK_FILES_FOR_robif2b::2dof "${_IMPORT_PREFIX}/lib/lib2dof.so" )

# Import target "robif2b::ethercat" for configuration ""
set_property(TARGET robif2b::ethercat APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robif2b::ethercat PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "soem"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libethercat.so"
  IMPORTED_SONAME_NOCONFIG "libethercat.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robif2b::ethercat )
list(APPEND _IMPORT_CHECK_FILES_FOR_robif2b::ethercat "${_IMPORT_PREFIX}/lib/libethercat.so" )

# Import target "robif2b::kelo" for configuration ""
set_property(TARGET robif2b::kelo APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(robif2b::kelo PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkelo.so"
  IMPORTED_SONAME_NOCONFIG "libkelo.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robif2b::kelo )
list(APPEND _IMPORT_CHECK_FILES_FOR_robif2b::kelo "${_IMPORT_PREFIX}/lib/libkelo.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
