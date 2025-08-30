# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mrsim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mrsim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mrsim_FOUND FALSE)
  elseif(NOT mrsim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mrsim_FOUND FALSE)
  endif()
  return()
endif()
set(_mrsim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mrsim_FIND_QUIETLY)
  message(STATUS "Found mrsim: 0.0.1 (${mrsim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mrsim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mrsim_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mrsim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mrsim_DIR}/${_extra}")
endforeach()
