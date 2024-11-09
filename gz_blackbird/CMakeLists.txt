cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

project(gz_blackbird)

# cmake_print_variables(PROJECT_SOURCE_DIR)


# we use std::optional here, we need 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g")
#set(Boost_USE_MULTITHREADED ON)


# Find the packages
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
#find_package(gz-msgs10 REQUIRED)


include(CMakePrintHelpers)

# Find Boost for subprocess
find_package(Boost 1.74.0 REQUIRED)
if(Boost_FOUND)
  message(STATUS "Boost found: ${Boost_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Boost not found")
endif()

# pybind11
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Include pybind11 AND PYTHON headers
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})

cmake_print_variables(CMAKE_CURRENT_SOURCE_DIR)


# EntityComponentManager Provider
add_library(ecm_provider include/ECMProvider.h src/ECMProvider.cpp)
target_include_directories(ecm_provider PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)

target_link_libraries(ecm_provider gz-sim8::gz-sim8)

# Joint Controller 
add_library(joint_controller include/EffortController.h src/EffortController.cpp)
target_include_directories(joint_controller PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)

target_link_libraries(joint_controller gz-sim8::gz-sim8)

# Contact Sensor
add_library(bipedal_contact include/BipedalContact.h src/BipedalContact.cpp)
target_include_directories(bipedal_contact PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)

target_link_libraries(bipedal_contact gz-sim8::gz-sim8)

# Robot State Updater
add_library(state_updater include/RobotStateUpdater.h src/RobotStateUpdater.cpp)
target_include_directories(state_updater PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)

target_link_libraries(state_updater gz-sim8::gz-sim8)

# TrainSimulator
add_library(train_simulator include/Simulator.h 
src/Simulator.cpp include/ECMProvider.h 
include/EffortController.h 
include/BipedalContact.h
include/RobotStateUpdater.h)


target_include_directories(train_simulator PUBLIC ${Boost_INCLUDE_DIRS} PRIVATE ${Python3_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS})
target_link_libraries(train_simulator 
ecm_provider 
joint_controller 
bipedal_contact
state_updater 
${Boost_LIBRARIES}
gz-math7) # link TrainSimulator with ECMProvider




# Add the source files
add_library(blackbird_rl MODULE src/bindings.cpp)

set_target_properties(blackbird_rl PROPERTIES PREFIX "") # python expects 'blackbird_rl.so'
target_include_directories(blackbird_rl PRIVATE ${Python3_INCLUDE_DIRS})

# Link pybind11
target_link_libraries(blackbird_rl PRIVATE pybind11::module train_simulator ${Python3_LIBRARIES})


# our executable/binary
add_executable(launch_empty_world src/main.cpp)
target_link_libraries(launch_empty_world train_simulator
gz-sim8::gz-sim8
${Python3_LIBRARIES})


# reset loop
add_executable(sim_loop src/reset_loop.cpp)
target_link_libraries(sim_loop train_simulator
gz-sim8::gz-sim8
${Python3_LIBRARIES})
