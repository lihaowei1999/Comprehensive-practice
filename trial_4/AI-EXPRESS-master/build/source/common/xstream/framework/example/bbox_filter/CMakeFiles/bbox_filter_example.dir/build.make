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
include source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/depend.make

# Include the progress variables for this target.
include source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/progress.make

# Include the compile flags for this target's objects.
include source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/flags.make

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/flags.make
source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o: ../source/common/xstream/framework/example/bbox_filter/src/method/bbox_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o -c /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/method/bbox_filter.cpp

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.i"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/method/bbox_filter.cpp > CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.i

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.s"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/method/bbox_filter.cpp -o CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.s

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.requires:

.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.requires

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.provides: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.requires
	$(MAKE) -f source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/build.make source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.provides.build
.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.provides

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.provides.build: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o


source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/flags.make
source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o: ../source/common/xstream/framework/example/bbox_filter/src/method_factory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o -c /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/method_factory.cpp

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.i"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/method_factory.cpp > CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.i

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.s"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/method_factory.cpp -o CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.s

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.requires:

.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.requires

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.provides: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.requires
	$(MAKE) -f source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/build.make source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.provides.build
.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.provides

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.provides.build: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o


source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/flags.make
source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o: ../source/common/xstream/framework/example/bbox_filter/src/bbox_filter_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o -c /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/bbox_filter_main.cpp

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.i"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/bbox_filter_main.cpp > CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.i

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.s"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter/src/bbox_filter_main.cpp -o CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.s

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.requires:

.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.requires

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.provides: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.requires
	$(MAKE) -f source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/build.make source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.provides.build
.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.provides

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.provides.build: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o


# Object files for target bbox_filter_example
bbox_filter_example_OBJECTS = \
"CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o" \
"CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o" \
"CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o"

# External object files for target bbox_filter_example
bbox_filter_example_EXTERNAL_OBJECTS =

bin/bbox_filter_example: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o
bin/bbox_filter_example: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o
bin/bbox_filter_example: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o
bin/bbox_filter_example: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/build.make
bin/bbox_filter_example: lib/libxstream.a
bin/bbox_filter_example: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../../../../../../bin/bbox_filter_example"
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bbox_filter_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/build: bin/bbox_filter_example

.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/build

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/requires: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method/bbox_filter.cpp.o.requires
source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/requires: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/method_factory.cpp.o.requires
source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/requires: source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/src/bbox_filter_main.cpp.o.requires

.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/requires

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/clean:
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter && $(CMAKE_COMMAND) -P CMakeFiles/bbox_filter_example.dir/cmake_clean.cmake
.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/clean

source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/depend:
	cd /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihaowei/docker/trial_4/AI-EXPRESS-master /home/lihaowei/docker/trial_4/AI-EXPRESS-master/source/common/xstream/framework/example/bbox_filter /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter /home/lihaowei/docker/trial_4/AI-EXPRESS-master/build/source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : source/common/xstream/framework/example/bbox_filter/CMakeFiles/bbox_filter_example.dir/depend
