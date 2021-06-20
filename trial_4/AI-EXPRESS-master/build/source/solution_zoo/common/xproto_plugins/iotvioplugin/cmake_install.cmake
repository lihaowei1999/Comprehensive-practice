# Install script for directory: /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/common/xproto_plugins/iotvioplugin

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
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//vioplugin//include/vioplugin/")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//vioplugin//include/vioplugin" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/common/xproto_plugins/iotvioplugin/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib" TYPE SHARED_LIBRARY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/lib/libvioplugin.so")
  if(EXISTS "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so"
         OLD_RPATH "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/x3_prebuilt/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/x3_prebuilt/lib/appsdk:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/x3_prebuilt/lib/appsdk/hbmedia:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/x3_prebuilt/lib/appsdk/hbbpu:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/ipc_tracking/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/bpu_predict/x3/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/jsoncpp/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/hobotlog/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/protobuf/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/gtest/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/opencv/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/libjpeg-turbo/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/libyuv/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/hobot/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/iou_based_mot/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/zlib/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/live555/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/xwarehouse/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/uWS/lib64:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/openssl/lib:/home/lihaowei/docker/trial_4/AI-EXPRESS-master/deps/zeroMQ/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output/vioplugin/lib/libvioplugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//vioplugin//config//")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/output//vioplugin//config/" TYPE DIRECTORY FILES "/home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/solution_zoo/common/xproto_plugins/iotvioplugin/configs/")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/solution_zoo/common/xproto_plugins/iotvioplugin/sample/cmake_install.cmake")
  include("/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/solution_zoo/common/xproto_plugins/iotvioplugin/src/iotviomanager/camera/plugin/cmake_install.cmake")

endif()

