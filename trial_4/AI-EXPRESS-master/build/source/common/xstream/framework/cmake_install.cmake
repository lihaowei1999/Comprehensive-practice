# Install script for directory: /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework

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
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxsdk/version.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxsdk/xstream_data.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxsdk/xstream_error.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxsdk/xstream_sdk.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxsdk" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxsdk/version.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxsdk/xstream_data.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxsdk/xstream_error.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxsdk/xstream_sdk.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream/json_key.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream/method_factory.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream/method.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream/simple_method.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream/profiler.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream/xstream_config.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/hobotxstream" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxstream/json_key.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxstream/method_factory.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxstream/method.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxstream/simple_method.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxstream/profiler.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/include/hobotxstream/xstream_config.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/dnncommon/DnnAsyncData.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/dnncommon/dnn_util.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/include/dnncommon" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/methods/DnnAsyncData/DnnAsyncData.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/methods/dnn_util/include/dnn_util.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/lib/libxstream.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/lib" TYPE STATIC_LIBRARY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/lib/libxstream.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/README.md")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream" TYPE FILE FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/README.md")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream//tutorials")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/tutorials/third_party")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xstream/tutorials" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/third_party")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/test/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/benchmark/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage1/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage2/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage3/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage4/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage5/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage6/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage7/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage8/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage9/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPostProcessMethod/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10/cmake_install.cmake")

endif()

