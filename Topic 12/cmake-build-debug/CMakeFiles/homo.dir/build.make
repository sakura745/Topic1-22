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
CMAKE_SOURCE_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 12"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/homo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/homo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/homo.dir/flags.make

CMakeFiles/homo.dir/virtual-billboard.cpp.o: CMakeFiles/homo.dir/flags.make
CMakeFiles/homo.dir/virtual-billboard.cpp.o: ../virtual-billboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/homo.dir/virtual-billboard.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/homo.dir/virtual-billboard.cpp.o -c "/home/sakura/Documents/000000FromO/Homework/Topic 12/virtual-billboard.cpp"

CMakeFiles/homo.dir/virtual-billboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/homo.dir/virtual-billboard.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sakura/Documents/000000FromO/Homework/Topic 12/virtual-billboard.cpp" > CMakeFiles/homo.dir/virtual-billboard.cpp.i

CMakeFiles/homo.dir/virtual-billboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/homo.dir/virtual-billboard.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sakura/Documents/000000FromO/Homework/Topic 12/virtual-billboard.cpp" -o CMakeFiles/homo.dir/virtual-billboard.cpp.s

# Object files for target homo
homo_OBJECTS = \
"CMakeFiles/homo.dir/virtual-billboard.cpp.o"

# External object files for target homo
homo_EXTERNAL_OBJECTS =

homo: CMakeFiles/homo.dir/virtual-billboard.cpp.o
homo: CMakeFiles/homo.dir/build.make
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: /usr/local/lib/libopencv_world.so.3.1.0
homo: CMakeFiles/homo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable homo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/homo.dir/build: homo

.PHONY : CMakeFiles/homo.dir/build

CMakeFiles/homo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/homo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/homo.dir/clean

CMakeFiles/homo.dir/depend:
	cd "/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sakura/Documents/000000FromO/Homework/Topic 12" "/home/sakura/Documents/000000FromO/Homework/Topic 12" "/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 12/cmake-build-debug/CMakeFiles/homo.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/homo.dir/depend
