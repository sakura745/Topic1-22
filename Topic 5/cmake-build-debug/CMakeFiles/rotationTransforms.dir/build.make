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
CMAKE_SOURCE_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 5"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/rotationTransforms.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rotationTransforms.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rotationTransforms.dir/flags.make

CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.o: CMakeFiles/rotationTransforms.dir/flags.make
CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.o: ../rotationTransforms.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.o -c "/home/sakura/Documents/000000FromO/Homework/Topic 5/rotationTransforms.cpp"

CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sakura/Documents/000000FromO/Homework/Topic 5/rotationTransforms.cpp" > CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.i

CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sakura/Documents/000000FromO/Homework/Topic 5/rotationTransforms.cpp" -o CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.s

# Object files for target rotationTransforms
rotationTransforms_OBJECTS = \
"CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.o"

# External object files for target rotationTransforms
rotationTransforms_EXTERNAL_OBJECTS =

rotationTransforms: CMakeFiles/rotationTransforms.dir/rotationTransforms.cpp.o
rotationTransforms: CMakeFiles/rotationTransforms.dir/build.make
rotationTransforms: CMakeFiles/rotationTransforms.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rotationTransforms"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rotationTransforms.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rotationTransforms.dir/build: rotationTransforms

.PHONY : CMakeFiles/rotationTransforms.dir/build

CMakeFiles/rotationTransforms.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rotationTransforms.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rotationTransforms.dir/clean

CMakeFiles/rotationTransforms.dir/depend:
	cd "/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sakura/Documents/000000FromO/Homework/Topic 5" "/home/sakura/Documents/000000FromO/Homework/Topic 5" "/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 5/cmake-build-debug/CMakeFiles/rotationTransforms.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/rotationTransforms.dir/depend

