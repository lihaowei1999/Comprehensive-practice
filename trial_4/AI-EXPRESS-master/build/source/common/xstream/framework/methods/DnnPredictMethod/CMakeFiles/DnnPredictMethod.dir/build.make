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
include source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/depend.make

# Include the progress variables for this target.
include source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/progress.make

# Include the compile flags for this target's objects.
include source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/flags.make

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/flags.make
source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o: ../source/common/xstream/framework/methods/DnnPredictMethod/src/DnnPredictMethod.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o -c /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/methods/DnnPredictMethod/src/DnnPredictMethod.cpp

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.i"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/methods/DnnPredictMethod/src/DnnPredictMethod.cpp > CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.i

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.s"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/methods/DnnPredictMethod/src/DnnPredictMethod.cpp -o CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.s

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.requires:

.PHONY : source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.requires

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.provides: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.requires
	$(MAKE) -f source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/build.make source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.provides.build
.PHONY : source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.provides

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.provides.build: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o


# Object files for target DnnPredictMethod
DnnPredictMethod_OBJECTS = \
"CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o"

# External object files for target DnnPredictMethod
DnnPredictMethod_EXTERNAL_OBJECTS =

lib/libDnnPredictMethod.a: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o
lib/libDnnPredictMethod.a: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/build.make
lib/libDnnPredictMethod.a: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../../../../lib/libDnnPredictMethod.a"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod && $(CMAKE_COMMAND) -P CMakeFiles/DnnPredictMethod.dir/cmake_clean_target.cmake
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DnnPredictMethod.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/build: lib/libDnnPredictMethod.a

.PHONY : source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/build

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/requires: source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/src/DnnPredictMethod.cpp.o.requires

.PHONY : source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/requires

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/clean:
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod && $(CMAKE_COMMAND) -P CMakeFiles/DnnPredictMethod.dir/cmake_clean.cmake
.PHONY : source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/clean

source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/depend:
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihaowei/docker/trial_4/AI-EXPRESS-master /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/methods/DnnPredictMethod /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : source/common/xstream/framework/methods/DnnPredictMethod/CMakeFiles/DnnPredictMethod.dir/depend
