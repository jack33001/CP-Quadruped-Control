#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadruped::quadruped_lib" for configuration ""
set_property(TARGET quadruped::quadruped_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(quadruped::quadruped_lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libquadruped_lib.so"
  IMPORTED_SONAME_NOCONFIG "libquadruped_lib.so"
  )

list(APPEND _cmake_import_check_targets quadruped::quadruped_lib )
list(APPEND _cmake_import_check_files_for_quadruped::quadruped_lib "${_IMPORT_PREFIX}/lib/libquadruped_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
