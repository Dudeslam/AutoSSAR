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
CMAKE_SOURCE_DIR = /mnt/hgfs/Thesis2019_code/workspace/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/hgfs/Thesis2019_code/workspace/build

# Include any dependencies generated for this target.
include drone_code/CMakeFiles/uav1_Coordination.dir/depend.make

# Include the progress variables for this target.
include drone_code/CMakeFiles/uav1_Coordination.dir/progress.make

# Include the compile flags for this target's objects.
include drone_code/CMakeFiles/uav1_Coordination.dir/flags.make

drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o: drone_code/CMakeFiles/uav1_Coordination.dir/flags.make
drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o: /mnt/hgfs/Thesis2019_code/workspace/src/drone_code/src/uav1_Coordination.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/hgfs/Thesis2019_code/workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o"
	cd /mnt/hgfs/Thesis2019_code/workspace/build/drone_code && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o -c /mnt/hgfs/Thesis2019_code/workspace/src/drone_code/src/uav1_Coordination.cpp

drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.i"
	cd /mnt/hgfs/Thesis2019_code/workspace/build/drone_code && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/hgfs/Thesis2019_code/workspace/src/drone_code/src/uav1_Coordination.cpp > CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.i

drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.s"
	cd /mnt/hgfs/Thesis2019_code/workspace/build/drone_code && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/hgfs/Thesis2019_code/workspace/src/drone_code/src/uav1_Coordination.cpp -o CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.s

drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.requires:

.PHONY : drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.requires

drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.provides: drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.requires
	$(MAKE) -f drone_code/CMakeFiles/uav1_Coordination.dir/build.make drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.provides.build
.PHONY : drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.provides

drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.provides.build: drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o


# Object files for target uav1_Coordination
uav1_Coordination_OBJECTS = \
"CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o"

# External object files for target uav1_Coordination
uav1_Coordination_EXTERNAL_OBJECTS =

/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: drone_code/CMakeFiles/uav1_Coordination.dir/build.make
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liboctomap.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liboctomath.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /home/simon/catkin_ws/devel/.private/mavros/lib/libmavros.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libGeographic.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libeigen_conversions.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /home/simon/catkin_ws/devel/.private/libmavconn/lib/libmavconn.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libclass_loader.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/libPocoFoundation.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libdl.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libroslib.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librospack.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liboctomap_ros.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liboctomap.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liboctomath.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libtf.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libtf2_ros.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libactionlib.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libmessage_filters.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libroscpp.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libxmlrpcpp.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libtf2.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librosconsole.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libroscpp_serialization.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librostime.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libcpp_common.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_system.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libpthread.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /home/simon/catkin_ws/devel/.private/mavros/lib/libmavros.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libGeographic.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libeigen_conversions.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /home/simon/catkin_ws/devel/.private/libmavconn/lib/libmavconn.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libclass_loader.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/libPocoFoundation.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libdl.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libroslib.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librospack.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/liboctomap_ros.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libtf.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libtf2_ros.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libactionlib.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libmessage_filters.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libroscpp.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libxmlrpcpp.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libtf2.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librosconsole.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libroscpp_serialization.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/librostime.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /opt/ros/melodic/lib/libcpp_common.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_system.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libpthread.so
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination: drone_code/CMakeFiles/uav1_Coordination.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/hgfs/Thesis2019_code/workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination"
	cd /mnt/hgfs/Thesis2019_code/workspace/build/drone_code && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uav1_Coordination.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
drone_code/CMakeFiles/uav1_Coordination.dir/build: /mnt/hgfs/Thesis2019_code/workspace/devel/lib/drone_code/uav1_Coordination

.PHONY : drone_code/CMakeFiles/uav1_Coordination.dir/build

drone_code/CMakeFiles/uav1_Coordination.dir/requires: drone_code/CMakeFiles/uav1_Coordination.dir/src/uav1_Coordination.cpp.o.requires

.PHONY : drone_code/CMakeFiles/uav1_Coordination.dir/requires

drone_code/CMakeFiles/uav1_Coordination.dir/clean:
	cd /mnt/hgfs/Thesis2019_code/workspace/build/drone_code && $(CMAKE_COMMAND) -P CMakeFiles/uav1_Coordination.dir/cmake_clean.cmake
.PHONY : drone_code/CMakeFiles/uav1_Coordination.dir/clean

drone_code/CMakeFiles/uav1_Coordination.dir/depend:
	cd /mnt/hgfs/Thesis2019_code/workspace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/hgfs/Thesis2019_code/workspace/src /mnt/hgfs/Thesis2019_code/workspace/src/drone_code /mnt/hgfs/Thesis2019_code/workspace/build /mnt/hgfs/Thesis2019_code/workspace/build/drone_code /mnt/hgfs/Thesis2019_code/workspace/build/drone_code/CMakeFiles/uav1_Coordination.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : drone_code/CMakeFiles/uav1_Coordination.dir/depend

