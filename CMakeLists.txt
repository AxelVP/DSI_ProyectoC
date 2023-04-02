##############################################################
###### PARA COMPILAR: > cmake .. -DCMAKE_ARGS="ahrs;mando;pid"
######                  cmake --build .
##############################################################


cmake_minimum_required(VERSION 3.12.4)


if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(DDSHelloWorld)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

# Set C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(-std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

message(STATUS "Configuring publisher/subscriber ...")


file(GLOB DDS_AHRS_SOURCES_CXX "topics/ahrs/*.cxx")
file(GLOB DDS_KILLSWITCH_SOURCES_CXX "topics/killswitch/*.cxx")
file(GLOB DDS_MANDO_SOURCES_CXX "topics/mando/*.cxx")
file(GLOB DDS_PID_SOURCES_CXX "topics/pid/*.cxx")

include_directories("/home/pi/Navio2-master/C++/Navio")
link_directories("/home/pi/Navio2-master/C++/Navio")

file(GLOB NAVIO2_SOURCES  "/home/pi/Navio2-master/C++/Navio/Navio2/*.cpp" "/home/pi/Navio2-master/C++/Navio/Navio2/*.h")
file(GLOB COMMON_SOURCES  "/home/pi/Navio2-master/C++/Navio/Common/*.cpp" "/home/pi/Navio2-master/C++/Navio/Common/*.h")

add_library(common ${COMMON_SOURCES})
add_library(navio2 ${NAVIO2_SOURCES})


set(ARGS_LIST "${CMAKE_ARGS}")

list(LENGTH ARGS_LIST len)
message(STATUS "Numero de elementos de la lista: " ${len})

foreach(X ${ARGS_LIST})
    MESSAGE("ARG: " ${X})

    if("${X}" MATCHES "pid")
        add_executable(pid pid/pid.cpp ${DDS_AHRS_SOURCES_CXX} ${DDS_MANDO_SOURCES_CXX} ${DDS_PID_SOURCES_CXX} pid/PIDControl.h pid/PIDControl.cpp)
        target_link_libraries(pid fastrtps fastcdr)
    endif()

    if("${X}" MATCHES "mando")
        add_executable(mando mando/mando.cpp ${DDS_MANDO_SOURCES_CXX} ${DDS_KILLSWITCH_SOURCES_CXX} ${COMMON_SOURCES} ${NAVIO2_SOURCES})
        target_link_libraries(mando fastrtps fastcdr common navio2)
    endif()

    if("${X}" MATCHES "motores")
        add_executable(motores motores/motores.cpp ${DDS_KILLSWITCH_SOURCES_CXX} ${DDS_PID_SOURCES_CXX} ${COMMON_SOURCES} ${NAVIO2_SOURCES})
        target_link_libraries(motores fastrtps fastcdr common navio2)
    endif()

    if("${X}" MATCHES "ahrs")
        add_executable(ahrs ahrs/ahrs.cpp ${DDS_AHRS_SOURCES_CXX})
        target_link_libraries(ahrs fastrtps fastcdr RTIMULib common navio2)
    endif()

    if("${X}" MATCHES "printer")
        add_executable(printer printer/printer.cpp ${DDS_KILLSWITCH_SOURCES_CXX} ${DDS_PID_SOURCES_CXX} ${DDS_MANDO_SOURCES_CXX} ${DDS_AHRS_SOURCES_CXX})
        target_link_libraries(printer fastrtps fastcdr)
    endif()

endforeach()


