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
CMAKE_SOURCE_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 1"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/text.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/text.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/text.dir/flags.make

CMakeFiles/text.dir/main_o.cpp.o: CMakeFiles/text.dir/flags.make
CMakeFiles/text.dir/main_o.cpp.o: ../main_o.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/text.dir/main_o.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/text.dir/main_o.cpp.o -c "/home/sakura/Documents/000000FromO/Homework/Topic 1/main_o.cpp"

CMakeFiles/text.dir/main_o.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/text.dir/main_o.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/sakura/Documents/000000FromO/Homework/Topic 1/main_o.cpp" > CMakeFiles/text.dir/main_o.cpp.i

CMakeFiles/text.dir/main_o.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/text.dir/main_o.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/sakura/Documents/000000FromO/Homework/Topic 1/main_o.cpp" -o CMakeFiles/text.dir/main_o.cpp.s

# Object files for target text
text_OBJECTS = \
"CMakeFiles/text.dir/main_o.cpp.o"

# External object files for target text
text_EXTERNAL_OBJECTS =

text: CMakeFiles/text.dir/main_o.cpp.o
text: CMakeFiles/text.dir/build.make
text: CMakeFiles/text.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable text"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/text.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/text.dir/build: text

.PHONY : CMakeFiles/text.dir/build

CMakeFiles/text.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/text.dir/cmake_clean.cmake
.PHONY : CMakeFiles/text.dir/clean

CMakeFiles/text.dir/depend:
	cd "/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sakura/Documents/000000FromO/Homework/Topic 1" "/home/sakura/Documents/000000FromO/Homework/Topic 1" "/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug" "/home/sakura/Documents/000000FromO/Homework/Topic 1/cmake-build-debug/CMakeFiles/text.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/text.dir/depend
