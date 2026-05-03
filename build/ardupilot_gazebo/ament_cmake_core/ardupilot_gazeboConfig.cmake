# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ardupilot_gazebo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ardupilot_gazebo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ardupilot_gazebo_FOUND FALSE)
  elseif(NOT ardupilot_gazebo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ardupilot_gazebo_FOUND FALSE)
  endif()
  return()
endif()
set(_ardupilot_gazebo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ardupilot_gazebo_FIND_QUIETLY)
  message(STATUS "Found ardupilot_gazebo: 0.0.0 (${ardupilot_gazebo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ardupilot_gazebo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ardupilot_gazebo_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ardupilot_gazebo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ardupilot_gazebo_DIR}/${_extra}")
endforeach()
