# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/sakura/Downloads/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/sakura/Downloads/clion-2019.2.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/PlotTrajectory.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PlotTrajectory.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PlotTrajectory.dir/flags.make

CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.o: CMakeFiles/PlotTrajectory.dir/flags.make
CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.o: ../PlotTrajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.o -c "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/PlotTrajectory.cpp"

CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/PlotTrajectory.cpp" > CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.i

CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/PlotTrajectory.cpp" -o CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.s

# Object files for target PlotTrajectory
PlotTrajectory_OBJECTS = \
"CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.o"

# External object files for target PlotTrajectory
PlotTrajectory_EXTERNAL_OBJECTS =

PlotTrajectory: CMakeFiles/PlotTrajectory.dir/PlotTrajectory.cpp.o
PlotTrajectory: CMakeFiles/PlotTrajectory.dir/build.make
PlotTrajectory: /home/sakura/shenlan/slambook/3rdparty/Pangolin/build/src/libpangolin.so
PlotTrajectory: /usr/local/lib/libSophus.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libGLU.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libGL.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libGLEW.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libwayland-client.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libwayland-egl.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libwayland-cursor.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libSM.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libICE.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libX11.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libXext.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libdc1394.so
PlotTrajectory: /usr/lib/libOpenNI.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libpng.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libz.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libjpeg.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libtiff.so
PlotTrajectory: /usr/lib/x86_64-linux-gnu/libIlmImf.so
PlotTrajectory: CMakeFiles/PlotTrajectory.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable PlotTrajectory"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PlotTrajectory.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PlotTrajectory.dir/build: PlotTrajectory

.PHONY : CMakeFiles/PlotTrajectory.dir/build

CMakeFiles/PlotTrajectory.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PlotTrajectory.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PlotTrajectory.dir/clean

CMakeFiles/PlotTrajectory.dir/depend:
	cd "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六" "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六" "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 8/作业8-小六/cmake-build-debug/CMakeFiles/PlotTrajectory.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/PlotTrajectory.dir/depend

