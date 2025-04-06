# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_closedloop_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED closedloop_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(closedloop_FOUND FALSE)
  elseif(NOT closedloop_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(closedloop_FOUND FALSE)
  endif()
  return()
endif()
set(_closedloop_CONFIG_INCLUDED TRUE)

# output package information
if(NOT closedloop_FIND_QUIETLY)
  message(STATUS "Found closedloop: 0.0.0 (${closedloop_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'closedloop' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${closedloop_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(closedloop_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${closedloop_DIR}/${_extra}")
endforeach()
