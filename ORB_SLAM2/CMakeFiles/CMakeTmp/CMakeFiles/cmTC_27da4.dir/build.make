# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp

# Include any dependencies generated for this target.
include CMakeFiles/cmTC_27da4.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cmTC_27da4.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cmTC_27da4.dir/flags.make

CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o: CMakeFiles/cmTC_27da4.dir/flags.make
CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o: /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/feature_tests.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=/home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o -c /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/feature_tests.cxx

CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.i: cmake_force
	@echo "Preprocessing CXX source to CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/feature_tests.cxx > CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.i

CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.s: cmake_force
	@echo "Compiling CXX source to assembly CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/feature_tests.cxx -o CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.s

CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.requires:

.PHONY : CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.requires

CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.provides: CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.requires
	$(MAKE) -f CMakeFiles/cmTC_27da4.dir/build.make CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.provides.build
.PHONY : CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.provides

CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.provides.build: CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o


# Object files for target cmTC_27da4
cmTC_27da4_OBJECTS = \
"CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o"

# External object files for target cmTC_27da4
cmTC_27da4_EXTERNAL_OBJECTS =

cmTC_27da4: CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o
cmTC_27da4: CMakeFiles/cmTC_27da4.dir/build.make
cmTC_27da4: CMakeFiles/cmTC_27da4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=/home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cmTC_27da4"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cmTC_27da4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cmTC_27da4.dir/build: cmTC_27da4

.PHONY : CMakeFiles/cmTC_27da4.dir/build

CMakeFiles/cmTC_27da4.dir/requires: CMakeFiles/cmTC_27da4.dir/feature_tests.cxx.o.requires

.PHONY : CMakeFiles/cmTC_27da4.dir/requires

CMakeFiles/cmTC_27da4.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cmTC_27da4.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cmTC_27da4.dir/clean

CMakeFiles/cmTC_27da4.dir/depend:
	cd /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp /home/maxrebo/Workspace/ros_ws/src/ORB_SLAM2/CMakeFiles/CMakeTmp/CMakeFiles/cmTC_27da4.dir/DependInfo.cmake
.PHONY : CMakeFiles/cmTC_27da4.dir/depend

