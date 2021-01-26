cmake_minimum_required(VERSION 2.8.12)

project(swarm_tracking)

# adiciona dependencias argos3 + lua

set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
include(ARGoSCheckQTOpenGL)
find_package(Lua53 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})
link_directories(${ARGOS_LIBRARY_DIRS})

# compila subdiretórios

add_subdirectory(footbot_tracking)
add_subdirectory(loop_functions)
