# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/aksel/Desktop/Code/simu

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aksel/Desktop/Code/simu/build

# Include any dependencies generated for this target.
include CMakeFiles/BallMazeGame.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/BallMazeGame.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/BallMazeGame.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BallMazeGame.dir/flags.make

CMakeFiles/BallMazeGame.dir/main.cpp.o: CMakeFiles/BallMazeGame.dir/flags.make
CMakeFiles/BallMazeGame.dir/main.cpp.o: ../main.cpp
CMakeFiles/BallMazeGame.dir/main.cpp.o: CMakeFiles/BallMazeGame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BallMazeGame.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BallMazeGame.dir/main.cpp.o -MF CMakeFiles/BallMazeGame.dir/main.cpp.o.d -o CMakeFiles/BallMazeGame.dir/main.cpp.o -c /home/aksel/Desktop/Code/simu/main.cpp

CMakeFiles/BallMazeGame.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallMazeGame.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Desktop/Code/simu/main.cpp > CMakeFiles/BallMazeGame.dir/main.cpp.i

CMakeFiles/BallMazeGame.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallMazeGame.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Desktop/Code/simu/main.cpp -o CMakeFiles/BallMazeGame.dir/main.cpp.s

CMakeFiles/BallMazeGame.dir/ball.cpp.o: CMakeFiles/BallMazeGame.dir/flags.make
CMakeFiles/BallMazeGame.dir/ball.cpp.o: ../ball.cpp
CMakeFiles/BallMazeGame.dir/ball.cpp.o: CMakeFiles/BallMazeGame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/BallMazeGame.dir/ball.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BallMazeGame.dir/ball.cpp.o -MF CMakeFiles/BallMazeGame.dir/ball.cpp.o.d -o CMakeFiles/BallMazeGame.dir/ball.cpp.o -c /home/aksel/Desktop/Code/simu/ball.cpp

CMakeFiles/BallMazeGame.dir/ball.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallMazeGame.dir/ball.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Desktop/Code/simu/ball.cpp > CMakeFiles/BallMazeGame.dir/ball.cpp.i

CMakeFiles/BallMazeGame.dir/ball.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallMazeGame.dir/ball.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Desktop/Code/simu/ball.cpp -o CMakeFiles/BallMazeGame.dir/ball.cpp.s

CMakeFiles/BallMazeGame.dir/wall.cpp.o: CMakeFiles/BallMazeGame.dir/flags.make
CMakeFiles/BallMazeGame.dir/wall.cpp.o: ../wall.cpp
CMakeFiles/BallMazeGame.dir/wall.cpp.o: CMakeFiles/BallMazeGame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/BallMazeGame.dir/wall.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BallMazeGame.dir/wall.cpp.o -MF CMakeFiles/BallMazeGame.dir/wall.cpp.o.d -o CMakeFiles/BallMazeGame.dir/wall.cpp.o -c /home/aksel/Desktop/Code/simu/wall.cpp

CMakeFiles/BallMazeGame.dir/wall.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallMazeGame.dir/wall.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Desktop/Code/simu/wall.cpp > CMakeFiles/BallMazeGame.dir/wall.cpp.i

CMakeFiles/BallMazeGame.dir/wall.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallMazeGame.dir/wall.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Desktop/Code/simu/wall.cpp -o CMakeFiles/BallMazeGame.dir/wall.cpp.s

CMakeFiles/BallMazeGame.dir/point3d.cpp.o: CMakeFiles/BallMazeGame.dir/flags.make
CMakeFiles/BallMazeGame.dir/point3d.cpp.o: ../point3d.cpp
CMakeFiles/BallMazeGame.dir/point3d.cpp.o: CMakeFiles/BallMazeGame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/BallMazeGame.dir/point3d.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BallMazeGame.dir/point3d.cpp.o -MF CMakeFiles/BallMazeGame.dir/point3d.cpp.o.d -o CMakeFiles/BallMazeGame.dir/point3d.cpp.o -c /home/aksel/Desktop/Code/simu/point3d.cpp

CMakeFiles/BallMazeGame.dir/point3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallMazeGame.dir/point3d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Desktop/Code/simu/point3d.cpp > CMakeFiles/BallMazeGame.dir/point3d.cpp.i

CMakeFiles/BallMazeGame.dir/point3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallMazeGame.dir/point3d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Desktop/Code/simu/point3d.cpp -o CMakeFiles/BallMazeGame.dir/point3d.cpp.s

CMakeFiles/BallMazeGame.dir/maze.cpp.o: CMakeFiles/BallMazeGame.dir/flags.make
CMakeFiles/BallMazeGame.dir/maze.cpp.o: ../maze.cpp
CMakeFiles/BallMazeGame.dir/maze.cpp.o: CMakeFiles/BallMazeGame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/BallMazeGame.dir/maze.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BallMazeGame.dir/maze.cpp.o -MF CMakeFiles/BallMazeGame.dir/maze.cpp.o.d -o CMakeFiles/BallMazeGame.dir/maze.cpp.o -c /home/aksel/Desktop/Code/simu/maze.cpp

CMakeFiles/BallMazeGame.dir/maze.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallMazeGame.dir/maze.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Desktop/Code/simu/maze.cpp > CMakeFiles/BallMazeGame.dir/maze.cpp.i

CMakeFiles/BallMazeGame.dir/maze.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallMazeGame.dir/maze.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Desktop/Code/simu/maze.cpp -o CMakeFiles/BallMazeGame.dir/maze.cpp.s

CMakeFiles/BallMazeGame.dir/updater.cpp.o: CMakeFiles/BallMazeGame.dir/flags.make
CMakeFiles/BallMazeGame.dir/updater.cpp.o: ../updater.cpp
CMakeFiles/BallMazeGame.dir/updater.cpp.o: CMakeFiles/BallMazeGame.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/BallMazeGame.dir/updater.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BallMazeGame.dir/updater.cpp.o -MF CMakeFiles/BallMazeGame.dir/updater.cpp.o.d -o CMakeFiles/BallMazeGame.dir/updater.cpp.o -c /home/aksel/Desktop/Code/simu/updater.cpp

CMakeFiles/BallMazeGame.dir/updater.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallMazeGame.dir/updater.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Desktop/Code/simu/updater.cpp > CMakeFiles/BallMazeGame.dir/updater.cpp.i

CMakeFiles/BallMazeGame.dir/updater.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallMazeGame.dir/updater.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Desktop/Code/simu/updater.cpp -o CMakeFiles/BallMazeGame.dir/updater.cpp.s

# Object files for target BallMazeGame
BallMazeGame_OBJECTS = \
"CMakeFiles/BallMazeGame.dir/main.cpp.o" \
"CMakeFiles/BallMazeGame.dir/ball.cpp.o" \
"CMakeFiles/BallMazeGame.dir/wall.cpp.o" \
"CMakeFiles/BallMazeGame.dir/point3d.cpp.o" \
"CMakeFiles/BallMazeGame.dir/maze.cpp.o" \
"CMakeFiles/BallMazeGame.dir/updater.cpp.o"

# External object files for target BallMazeGame
BallMazeGame_EXTERNAL_OBJECTS =

BallMazeGame: CMakeFiles/BallMazeGame.dir/main.cpp.o
BallMazeGame: CMakeFiles/BallMazeGame.dir/ball.cpp.o
BallMazeGame: CMakeFiles/BallMazeGame.dir/wall.cpp.o
BallMazeGame: CMakeFiles/BallMazeGame.dir/point3d.cpp.o
BallMazeGame: CMakeFiles/BallMazeGame.dir/maze.cpp.o
BallMazeGame: CMakeFiles/BallMazeGame.dir/updater.cpp.o
BallMazeGame: CMakeFiles/BallMazeGame.dir/build.make
BallMazeGame: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so.2.5.1
BallMazeGame: /usr/lib/x86_64-linux-gnu/libsfml-window.so.2.5.1
BallMazeGame: /usr/lib/x86_64-linux-gnu/libsfml-system.so.2.5.1
BallMazeGame: CMakeFiles/BallMazeGame.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aksel/Desktop/Code/simu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable BallMazeGame"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BallMazeGame.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BallMazeGame.dir/build: BallMazeGame
.PHONY : CMakeFiles/BallMazeGame.dir/build

CMakeFiles/BallMazeGame.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BallMazeGame.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BallMazeGame.dir/clean

CMakeFiles/BallMazeGame.dir/depend:
	cd /home/aksel/Desktop/Code/simu/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aksel/Desktop/Code/simu /home/aksel/Desktop/Code/simu /home/aksel/Desktop/Code/simu/build /home/aksel/Desktop/Code/simu/build /home/aksel/Desktop/Code/simu/build/CMakeFiles/BallMazeGame.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BallMazeGame.dir/depend

