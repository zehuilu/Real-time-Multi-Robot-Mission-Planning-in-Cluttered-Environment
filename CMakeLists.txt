cmake_minimum_required(VERSION 3.12)
project(DrMaMP)

if(UNIX AND NOT APPLE)
    set(CMAKE_C_COMPILER "gcc")
    set(CMAKE_CXX_COMPILER "g++")
elseif(APPLE)
    set(CMAKE_C_COMPILER "clang")
    set(CMAKE_CXX_COMPILER "clang++")
else()
    message( WARNING "Only support LINUX and APPLE for now." )
endif()

# For solving TSP problem via Google OR-Tools
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${ORTOOLS_ROOT})
find_package(ortools REQUIRED)

# For python binding
find_package(pybind11 REQUIRED)
find_package(Python COMPONENTS Interpreter Development)

# For multi threading
find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include header directories
include_directories(
    # supress warnings for or-tools
    SYSTEM ${ORTOOLS_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include)

# add_definitions(-DUSE_GLOP)

# find all the header files
file(GLOB THETA_HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/*.hpp)
file(GLOB THETA_HEADER_FILES_H ${CMAKE_SOURCE_DIR}/externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/*.h)
file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)
file(GLOB HEADER_FILES_H ${CMAKE_SOURCE_DIR}/include/*.h)

# add executable files
add_executable(test_or_tools ${CMAKE_SOURCE_DIR}/test/test_or_tools.cpp)
target_link_libraries(test_or_tools ${ORTOOLS_LIBRARIES})

add_executable(test_tri ${CMAKE_SOURCE_DIR}/test/test_tri.cpp)

add_executable(test_solve_cpp test/test_solve_cpp.cpp ${HEADER_FILES_HPP} ${HEADER_FILES_H} ${THETA_HEADER_FILES_HPP} ${THETA_HEADER_FILES_H})
target_link_libraries(test_solve_cpp ${ORTOOLS_LIBRARIES})

# add executable test files
add_executable(test_k_means test/test_k_means.cpp ${HEADER_FILES_HPP})

# Build C++ library for Python, the solver
pybind11_add_module(DrMaMP src/DrMaMP.cpp)
target_link_libraries(DrMaMP PRIVATE ${ORTOOLS_LIBRARIES} pthread)

# Build C++ library for Python, K-means clustering
pybind11_add_module(K_MEANS src/K_MEANS.cpp)
target_link_libraries(K_MEANS PRIVATE pthread)
