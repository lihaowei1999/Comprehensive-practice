# Install script for directory: /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework

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
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/manager/msg_manager.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/manager" TYPE FILE FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/manager/msg_manager.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/plugin/xplugin.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/plugin/xpluginasync.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/plugin" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/plugin/xplugin.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/plugin/xpluginasync.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/message/pluginflow/flowmsg.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/message/pluginflow/msg_registry.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/message/pluginflow" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/message/pluginflow/flowmsg.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/message/pluginflow/msg_registry.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/threads/threadpool.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/threads" TYPE FILE FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/threads/threadpool.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/utils/singleton.h;/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/utils/profile.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/include/xproto/utils" TYPE FILE FILES
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/utils/singleton.h"
    "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/include/xproto/utils/profile.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/README.md")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto" TYPE FILE FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/README.md")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//xproto//document")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//xproto/" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/document")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/lib/libxproto.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/xproto/lib" TYPE STATIC_LIBRARY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/lib/libxproto.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//xproto//tutorials")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//xproto/" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/tutorials")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//xproto/tutorials/third_party")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//xproto/tutorials" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xproto/framework/third_party")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xproto/framework/test/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xproto/framework/tutorials/stage1/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xproto/framework/tutorials/stage3/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xproto/framework/tutorials/stage2/cmake_install.cmake")

endif()

