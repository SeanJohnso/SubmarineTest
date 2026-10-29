# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_jit_sonar_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED jit_sonar_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(jit_sonar_FOUND FALSE)
  elseif(NOT jit_sonar_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(jit_sonar_FOUND FALSE)
  endif()
  return()
endif()
set(_jit_sonar_CONFIG_INCLUDED TRUE)

# output package information
if(NOT jit_sonar_FIND_QUIETLY)
  message(STATUS "Found jit_sonar: 0.1.0 (${jit_sonar_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'jit_sonar' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${jit_sonar_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(jit_sonar_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${jit_sonar_DIR}/${_extra}")
endforeach()
