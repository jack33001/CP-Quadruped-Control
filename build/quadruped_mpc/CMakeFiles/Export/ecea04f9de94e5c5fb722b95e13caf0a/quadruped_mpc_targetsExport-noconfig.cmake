#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadruped_mpc::quadruped_mpc_controllers" for configuration ""
set_property(TARGET quadruped_mpc::quadruped_mpc_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(quadruped_mpc::quadruped_mpc_controllers PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libquadruped_mpc_controllers.so"
  IMPORTED_SONAME_NOCONFIG "libquadruped_mpc_controllers.so"
  )

list(APPEND _cmake_import_check_targets quadruped_mpc::quadruped_mpc_controllers )
list(APPEND _cmake_import_check_files_for_quadruped_mpc::quadruped_mpc_controllers "${_IMPORT_PREFIX}/lib/libquadruped_mpc_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
