# Install script for directory: /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/lib/libvehicle_snap_method.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/lib" TYPE STATIC_LIBRARY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/lib/libvehicle_snap_method.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_snap_strategy_api.hpp;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_snap_strategy_namespace.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method/include/vehicle_snap_strategy/vehicle_snap_strategy_api.hpp"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method/include/vehicle_snap_strategy_namespace.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_common_utility/data_type.hpp;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_common_utility/utils.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_common_utility" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method/include/vehicle_common_utility/data_type.hpp"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method/include/vehicle_common_utility/utils.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_match_strategy/config_params_type.hpp;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_match_strategy/pairs_match_api.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vehicle_snap_method/include/vehicle_snap_method/vehicle_match_strategy" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method/include/vehicle_match_strategy/config_params_type.hpp"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/xstream/methods/vehicle_snap_method/include/vehicle_match_strategy/pairs_match_api.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/solution_zoo/xstream/methods/vehicle_snap_method/test/cmake_install.cmake")

endif()

