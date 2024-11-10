#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadruped_mpc::quadruped_mpc_lib" for configuration ""
set_property(TARGET quadruped_mpc::quadruped_mpc_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(quadruped_mpc::quadruped_mpc_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libquadruped_mpc_lib.a"
  )

list(APPEND _cmake_import_check_targets quadruped_mpc::quadruped_mpc_lib )
list(APPEND _cmake_import_check_files_for_quadruped_mpc::quadruped_mpc_lib "${_IMPORT_PREFIX}/lib/libquadruped_mpc_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
