# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_quadruped_sim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED quadruped_sim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(quadruped_sim_FOUND FALSE)
  elseif(NOT quadruped_sim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(quadruped_sim_FOUND FALSE)
  endif()
  return()
endif()
set(_quadruped_sim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT quadruped_sim_FIND_QUIETLY)
  message(STATUS "Found quadruped_sim: 0.0.0 (${quadruped_sim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'quadruped_sim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT quadruped_sim_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(quadruped_sim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${quadruped_sim_DIR}/${_extra}")
endforeach()
