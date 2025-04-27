# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_forward_kinematics_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED forward_kinematics_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(forward_kinematics_FOUND FALSE)
  elseif(NOT forward_kinematics_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(forward_kinematics_FOUND FALSE)
  endif()
  return()
endif()
set(_forward_kinematics_CONFIG_INCLUDED TRUE)

# output package information
if(NOT forward_kinematics_FIND_QUIETLY)
  message(STATUS "Found forward_kinematics: 0.0.0 (${forward_kinematics_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'forward_kinematics' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${forward_kinematics_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(forward_kinematics_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${forward_kinematics_DIR}/${_extra}")
endforeach()
