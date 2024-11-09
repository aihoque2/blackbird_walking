# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312

# Include any dependencies generated for this target.
include CMakeFiles/train_simulator.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/train_simulator.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/train_simulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/train_simulator.dir/flags.make

CMakeFiles/train_simulator.dir/src/Simulator.cpp.o: CMakeFiles/train_simulator.dir/flags.make
CMakeFiles/train_simulator.dir/src/Simulator.cpp.o: /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/src/Simulator.cpp
CMakeFiles/train_simulator.dir/src/Simulator.cpp.o: CMakeFiles/train_simulator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/train_simulator.dir/src/Simulator.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/train_simulator.dir/src/Simulator.cpp.o -MF CMakeFiles/train_simulator.dir/src/Simulator.cpp.o.d -o CMakeFiles/train_simulator.dir/src/Simulator.cpp.o -c /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/src/Simulator.cpp

CMakeFiles/train_simulator.dir/src/Simulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/train_simulator.dir/src/Simulator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/src/Simulator.cpp > CMakeFiles/train_simulator.dir/src/Simulator.cpp.i

CMakeFiles/train_simulator.dir/src/Simulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/train_simulator.dir/src/Simulator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/src/Simulator.cpp -o CMakeFiles/train_simulator.dir/src/Simulator.cpp.s

# Object files for target train_simulator
train_simulator_OBJECTS = \
"CMakeFiles/train_simulator.dir/src/Simulator.cpp.o"

# External object files for target train_simulator
train_simulator_EXTERNAL_OBJECTS =

libtrain_simulator.a: CMakeFiles/train_simulator.dir/src/Simulator.cpp.o
libtrain_simulator.a: CMakeFiles/train_simulator.dir/build.make
libtrain_simulator.a: CMakeFiles/train_simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libtrain_simulator.a"
	$(CMAKE_COMMAND) -P CMakeFiles/train_simulator.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/train_simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/train_simulator.dir/build: libtrain_simulator.a
.PHONY : CMakeFiles/train_simulator.dir/build

CMakeFiles/train_simulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/train_simulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/train_simulator.dir/clean

CMakeFiles/train_simulator.dir/depend:
	cd /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312 /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312 /home/ahoque245/Projects/simulations/blackbird_walking/gz_blackbird/build/temp.linux-x86_64-cpython-312/CMakeFiles/train_simulator.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/train_simulator.dir/depend

