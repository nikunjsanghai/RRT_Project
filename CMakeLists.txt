cmake_minimum_required(VERSION 3.10)
project(RRT_Project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Set the SFML directory to help CMake find SFMLConfig.cmake
set(SFML_DIR "/usr/lib/x86_64-linux-gnu/cmake/SFML")

# Find SFML components
find_package(SFML 2.5 COMPONENTS graphics audio window system REQUIRED)
if(NOT SFML_FOUND)
  message(FATAL_ERROR "SFML NOT FOUND")
endif()

# Find Google Test
find_package(GTest REQUIRED)
if(NOT GTEST_FOUND)
  message(FATAL_ERROR "Google Test not found.")
endif()

# Find fmt library (required for spdlog)
find_package(fmt REQUIRED)

# Find spdlog library
find_package(spdlog REQUIRED)

# Prometheus Client
find_package(prometheus-cpp CONFIG REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

link_directories(/usr/lib/x86_64-linux-gnu)

# Add executable for the main application
add_executable(rrt_3d src/main.cpp)

# Link SFML, Prometheus, spdlog, and fmt libraries
target_link_libraries(rrt_3d
  sfml-graphics sfml-window sfml-system sfml-audio pthread
  prometheus-cpp::core
  prometheus-cpp::push
  prometheus-cpp::pull
  spdlog::spdlog  # Link spdlog
  fmt             # Link fmt library for spdlog
)

# Add a second executable for the tests
add_executable(rrt_3d_tests src/test_main.cpp)

# Link Google Test, SFML, Prometheus, spdlog, and fmt libraries to the test executable
target_link_libraries(rrt_3d_tests
  GTest::gtest GTest::gtest_main pthread
  sfml-graphics sfml-window sfml-system sfml-audio
  prometheus-cpp::core
  prometheus-cpp::push
  prometheus-cpp::pull
  spdlog::spdlog  # Link spdlog for testing
  fmt             # Link fmt library for spdlog
)

# Enable testing
enable_testing()
add_test(NAME RRT_3D_Tests COMMAND rrt_3d_tests)
