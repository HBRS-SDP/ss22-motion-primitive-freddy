# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/barath/sdp/robif2b

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/barath/sdp/robif2b/build

# Include any dependencies generated for this target.
include src/nbx/2dof/CMakeFiles/2dof.dir/depend.make

# Include the progress variables for this target.
include src/nbx/2dof/CMakeFiles/2dof.dir/progress.make

# Include the compile flags for this target's objects.
include src/nbx/2dof/CMakeFiles/2dof.dir/flags.make

src/nbx/2dof/CMakeFiles/2dof.dir/2dof.c.o: src/nbx/2dof/CMakeFiles/2dof.dir/flags.make
src/nbx/2dof/CMakeFiles/2dof.dir/2dof.c.o: ../src/nbx/2dof/2dof.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/barath/sdp/robif2b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/nbx/2dof/CMakeFiles/2dof.dir/2dof.c.o"
	cd /home/barath/sdp/robif2b/build/src/nbx/2dof && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/2dof.dir/2dof.c.o   -c /home/barath/sdp/robif2b/src/nbx/2dof/2dof.c

src/nbx/2dof/CMakeFiles/2dof.dir/2dof.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/2dof.dir/2dof.c.i"
	cd /home/barath/sdp/robif2b/build/src/nbx/2dof && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/barath/sdp/robif2b/src/nbx/2dof/2dof.c > CMakeFiles/2dof.dir/2dof.c.i

src/nbx/2dof/CMakeFiles/2dof.dir/2dof.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/2dof.dir/2dof.c.s"
	cd /home/barath/sdp/robif2b/build/src/nbx/2dof && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/barath/sdp/robif2b/src/nbx/2dof/2dof.c -o CMakeFiles/2dof.dir/2dof.c.s

# Object files for target 2dof
2dof_OBJECTS = \
"CMakeFiles/2dof.dir/2dof.c.o"

# External object files for target 2dof
2dof_EXTERNAL_OBJECTS =

src/nbx/2dof/lib2dof.so: src/nbx/2dof/CMakeFiles/2dof.dir/2dof.c.o
src/nbx/2dof/lib2dof.so: src/nbx/2dof/CMakeFiles/2dof.dir/build.make
src/nbx/2dof/lib2dof.so: src/nbx/2dof/CMakeFiles/2dof.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/barath/sdp/robif2b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library lib2dof.so"
	cd /home/barath/sdp/robif2b/build/src/nbx/2dof && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/2dof.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/nbx/2dof/CMakeFiles/2dof.dir/build: src/nbx/2dof/lib2dof.so

.PHONY : src/nbx/2dof/CMakeFiles/2dof.dir/build

src/nbx/2dof/CMakeFiles/2dof.dir/clean:
	cd /home/barath/sdp/robif2b/build/src/nbx/2dof && $(CMAKE_COMMAND) -P CMakeFiles/2dof.dir/cmake_clean.cmake
.PHONY : src/nbx/2dof/CMakeFiles/2dof.dir/clean

src/nbx/2dof/CMakeFiles/2dof.dir/depend:
	cd /home/barath/sdp/robif2b/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/barath/sdp/robif2b /home/barath/sdp/robif2b/src/nbx/2dof /home/barath/sdp/robif2b/build /home/barath/sdp/robif2b/build/src/nbx/2dof /home/barath/sdp/robif2b/build/src/nbx/2dof/CMakeFiles/2dof.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/nbx/2dof/CMakeFiles/2dof.dir/depend

