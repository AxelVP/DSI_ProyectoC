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
CMAKE_SOURCE_DIR = /home/pi/aplicacion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/aplicacion/build

# Include any dependencies generated for this target.
include CMakeFiles/mando.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mando.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mando.dir/flags.make

CMakeFiles/mando.dir/mando/mando.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/mando/mando.cpp.o: ../mando/mando.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mando.dir/mando/mando.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/mando/mando.cpp.o -c /home/pi/aplicacion/mando/mando.cpp

CMakeFiles/mando.dir/mando/mando.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/mando/mando.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/aplicacion/mando/mando.cpp > CMakeFiles/mando.dir/mando/mando.cpp.i

CMakeFiles/mando.dir/mando/mando.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/mando/mando.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/aplicacion/mando/mando.cpp -o CMakeFiles/mando.dir/mando/mando.cpp.s

CMakeFiles/mando.dir/topics/mando/Mando.cxx.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/topics/mando/Mando.cxx.o: ../topics/mando/Mando.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mando.dir/topics/mando/Mando.cxx.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/topics/mando/Mando.cxx.o -c /home/pi/aplicacion/topics/mando/Mando.cxx

CMakeFiles/mando.dir/topics/mando/Mando.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/topics/mando/Mando.cxx.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/aplicacion/topics/mando/Mando.cxx > CMakeFiles/mando.dir/topics/mando/Mando.cxx.i

CMakeFiles/mando.dir/topics/mando/Mando.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/topics/mando/Mando.cxx.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/aplicacion/topics/mando/Mando.cxx -o CMakeFiles/mando.dir/topics/mando/Mando.cxx.s

CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.o: ../topics/mando/MandoPubSubTypes.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.o -c /home/pi/aplicacion/topics/mando/MandoPubSubTypes.cxx

CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/aplicacion/topics/mando/MandoPubSubTypes.cxx > CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.i

CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/aplicacion/topics/mando/MandoPubSubTypes.cxx -o CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.s

CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.o: ../topics/killswitch/Killswitch.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.o -c /home/pi/aplicacion/topics/killswitch/Killswitch.cxx

CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/aplicacion/topics/killswitch/Killswitch.cxx > CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.i

CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/aplicacion/topics/killswitch/Killswitch.cxx -o CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.s

CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.o: ../topics/killswitch/KillswitchPubSubTypes.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.o -c /home/pi/aplicacion/topics/killswitch/KillswitchPubSubTypes.cxx

CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/aplicacion/topics/killswitch/KillswitchPubSubTypes.cxx > CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.i

CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/aplicacion/topics/killswitch/KillswitchPubSubTypes.cxx -o CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.o: /home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.o -c /home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.o: /home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.o -c /home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.o: /home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.o -c /home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.o: /home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.o -c /home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.o: /home/pi/Navio2-master/C++/Navio/Common/Util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.o -c /home/pi/Navio2-master/C++/Navio/Common/Util.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Common/Util.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Common/Util.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.o: /home/pi/Navio2-master/C++/Navio/Common/gpio.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.o -c /home/pi/Navio2-master/C++/Navio/Common/gpio.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Common/gpio.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Common/gpio.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.s

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.o: CMakeFiles/mando.dir/flags.make
CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.o: /home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.o -c /home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp > CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.i

CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp -o CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.s

# Object files for target mando
mando_OBJECTS = \
"CMakeFiles/mando.dir/mando/mando.cpp.o" \
"CMakeFiles/mando.dir/topics/mando/Mando.cxx.o" \
"CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.o" \
"CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.o" \
"CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.o" \
"CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.o"

# External object files for target mando
mando_EXTERNAL_OBJECTS =

mando: CMakeFiles/mando.dir/mando/mando.cpp.o
mando: CMakeFiles/mando.dir/topics/mando/Mando.cxx.o
mando: CMakeFiles/mando.dir/topics/mando/MandoPubSubTypes.cxx.o
mando: CMakeFiles/mando.dir/topics/killswitch/Killswitch.cxx.o
mando: CMakeFiles/mando.dir/topics/killswitch/KillswitchPubSubTypes.cxx.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/I2Cdev.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MPU9250.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/MS5611.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Ublox.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/Util.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Common/gpio.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/ADC_Navio2.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/LSM9DS1.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/Led_Navio2.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/PWM.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCInput_Navio2.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RCOutput_Navio2.cpp.o
mando: CMakeFiles/mando.dir/home/pi/Navio2-master/C++/Navio/Navio2/RGBled.cpp.o
mando: CMakeFiles/mando.dir/build.make
mando: /usr/local/lib/libfastrtps.so.2.8.0
mando: /home/pi/Fast-DDS/install/fastcdr/lib/libfastcdr.so.1.0.25
mando: libcommon.a
mando: libnavio2.a
mando: /usr/local/lib/libfoonathan_memory-0.7.1.a
mando: /usr/lib/arm-linux-gnueabihf/libtinyxml2.so
mando: /usr/lib/arm-linux-gnueabihf/libtinyxml2.so
mando: /usr/lib/arm-linux-gnueabihf/libssl.so
mando: /usr/lib/arm-linux-gnueabihf/libcrypto.so
mando: CMakeFiles/mando.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/aplicacion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Linking CXX executable mando"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mando.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mando.dir/build: mando

.PHONY : CMakeFiles/mando.dir/build

CMakeFiles/mando.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mando.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mando.dir/clean

CMakeFiles/mando.dir/depend:
	cd /home/pi/aplicacion/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/aplicacion /home/pi/aplicacion /home/pi/aplicacion/build /home/pi/aplicacion/build /home/pi/aplicacion/build/CMakeFiles/mando.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mando.dir/depend

