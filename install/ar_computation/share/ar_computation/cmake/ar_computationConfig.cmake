# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ar_computation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ar_computation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ar_computation_FOUND FALSE)
  elseif(NOT ar_computation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ar_computation_FOUND FALSE)
  endif()
  return()
endif()
set(_ar_computation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ar_computation_FIND_QUIETLY)
  message(STATUS "Found ar_computation: 0.0.0 (${ar_computation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ar_computation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ar_computation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ar_computation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ar_computation_DIR}/${_extra}")
endforeach()
