# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/amir/Desktop/ardrone_simulator/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amir/Desktop/ardrone_simulator/build

# Include any dependencies generated for this target.
include ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/depend.make

# Include the progress variables for this target.
include ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/progress.make

# Include the compile flags for this target's objects.
include ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/flags.make

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/flags.make
ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o: /home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/src/test_trajectory.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/amir/Desktop/ardrone_simulator/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o"
	cd /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o -c /home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/src/test_trajectory.cpp

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.i"
	cd /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/src/test_trajectory.cpp > CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.i

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.s"
	cd /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/src/test_trajectory.cpp -o CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.s

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.requires:
.PHONY : ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.requires

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.provides: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.requires
	$(MAKE) -f ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/build.make ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.provides.build
.PHONY : ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.provides

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.provides.build: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o

# Object files for target test_trajectory
test_trajectory_OBJECTS = \
"CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o"

# External object files for target test_trajectory
test_trajectory_EXTERNAL_OBJECTS =

/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/build.make
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libcamera_info_manager.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libcamera_calibration_parsers.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libgazebo_ros_api_plugin.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libgazebo_ros_paths_plugin.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libtf.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libtf2_ros.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libactionlib.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libtf2.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libimage_transport.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libmessage_filters.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libclass_loader.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/libPocoFoundation.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libdl.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libroslib.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/librospack.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libroscpp.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/librosconsole.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/liblog4cxx.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/librostime.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /opt/ros/indigo/lib/libcpp_common.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory"
	cd /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_trajectory.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/build: /home/amir/Desktop/ardrone_simulator/devel/lib/cvg_sim_gazebo_plugins/test_trajectory
.PHONY : ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/build

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/requires: ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/src/test_trajectory.cpp.o.requires
.PHONY : ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/requires

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/clean:
	cd /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins && $(CMAKE_COMMAND) -P CMakeFiles/test_trajectory.dir/cmake_clean.cmake
.PHONY : ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/clean

ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/depend:
	cd /home/amir/Desktop/ardrone_simulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amir/Desktop/ardrone_simulator/src /home/amir/Desktop/ardrone_simulator/src/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins /home/amir/Desktop/ardrone_simulator/build /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins /home/amir/Desktop/ardrone_simulator/build/ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ardrone_simulator_gazebo7/cvg_sim_gazebo_plugins/CMakeFiles/test_trajectory.dir/depend

