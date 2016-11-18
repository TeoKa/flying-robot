cmake_minimum_required(VERSION 2.8.9)
project(LIRMM_flying_machine_mini_project)

#Bring the headers, such as robot.hpp into the project
include_directories(include)

#Can manually add the sources using the set command as follows:
#set(SOURCES src/main.cpp src/X.cpp)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")

add_executable(flyingRobot ${SOURCES})
