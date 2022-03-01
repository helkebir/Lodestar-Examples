cmake_minimum_required(VERSION 3.21)
project(lsExample)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include(FetchContent)

FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) # or whatever tag you want

FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

find_package(Lodestar CONFIG REQUIRED)

find_path(GINAC_INCLUDE_DIR ginac/ginac.h /usr/local/include /usr/include)
find_library(GINAC_LIBRARY ginac /usr/local/lib /usr/lib)

find_path(CLN_INCLUDE_DIR cln/real.h /usr/local/include /usr/include)
find_library(CLN_LIBRARY cln /usr/local/lib /usr/lib)

find_path(IRIMAGER_INCLUDE_DIR libirimager/IRImager.h /usr/local/include /usr/include)
find_library(IRIMAGER_LIBRARY irdirectsdk /usr/local/lib /usr/lib)

include_directories(${GINAC_INCLUDE_DIR} ${CLN_INCLUDE_DIR} ${IRIMAGER_INCLUDE_DIR})
link_directories(/usr/lib)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(fuzzyPIDSL)

find_package(Franka REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

file(GLOB_RECURSE SL_HEADERS "fuzzyPIDSL/*.h")
file(GLOB_RECURSE SL_SOURCES "fuzzyPIDSL/*.cpp")

add_executable(lsExample main.cpp ThermographerBlock.cpp ThermographerBlock.hpp PandaArmBlock.cpp PandaArmBlock.hpp  arm/ArmConnectionStatus.hpp arm/PandaState.cpp arm/PandaState.hpp arm/Panda.cpp arm/Panda.hpp FuzzyPIDExample.cpp FuzzyPIDExample.hpp AntiWindupPID.cpp AntiWindupPID.hpp PandaArmVelBlock.cpp PandaArmVelBlock.hpp ${SL_SOURCES} ${SL_HEADERS})
target_link_libraries(lsExample Lodestar)
target_link_libraries(lsExample ${GINAC_LIBRARY})
target_link_libraries(lsExample ${CLN_LIBRARY})
target_link_libraries(lsExample ${IRIMAGER_LIBRARY})
target_link_libraries(lsExample pthread usb-1.0 udev Franka::Franka Threads::Threads)
target_link_libraries(lsExample matplot)
target_link_libraries( lsExample ${OpenCV_LIBS} )