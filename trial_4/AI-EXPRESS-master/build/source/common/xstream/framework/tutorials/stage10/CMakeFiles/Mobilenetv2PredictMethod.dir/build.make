# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lihaowei/docker/trial_4/AI-EXPRESS-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build

# Include any dependencies generated for this target.
include source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/depend.make

# Include the progress variables for this target.
include source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/progress.make

# Include the compile flags for this target's objects.
include source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/flags.make

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/flags.make
source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o: ../source/common/xstream/framework/tutorials/stage10/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o -c /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/tutorials/stage10/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.i"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/tutorials/stage10/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc > CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.i

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.s"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/tutorials/stage10/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc -o CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.s

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.requires:

.PHONY : source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.requires

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.provides: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.requires
	$(MAKE) -f source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/build.make source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.provides.build
.PHONY : source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.provides

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.provides.build: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o


# Object files for target Mobilenetv2PredictMethod
Mobilenetv2PredictMethod_OBJECTS = \
"CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o"

# External object files for target Mobilenetv2PredictMethod
Mobilenetv2PredictMethod_EXTERNAL_OBJECTS =

lib/libMobilenetv2PredictMethod.a: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o
lib/libMobilenetv2PredictMethod.a: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/build.make
lib/libMobilenetv2PredictMethod.a: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../../../../lib/libMobilenetv2PredictMethod.a"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 && $(CMAKE_COMMAND) -P CMakeFiles/Mobilenetv2PredictMethod.dir/cmake_clean_target.cmake
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Mobilenetv2PredictMethod.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/build: lib/libMobilenetv2PredictMethod.a

.PHONY : source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/build

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/requires: source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/method/Mobilenetv2PredictMethod/mobilenetv2_predict_method.cc.o.requires

.PHONY : source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/requires

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/clean:
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 && $(CMAKE_COMMAND) -P CMakeFiles/Mobilenetv2PredictMethod.dir/cmake_clean.cmake
.PHONY : source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/clean

source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/depend:
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihaowei/docker/trial_4/AI-EXPRESS-master /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/tutorials/stage10 /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10 /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : source/common/xstream/framework/tutorials/stage10/CMakeFiles/Mobilenetv2PredictMethod.dir/depend

