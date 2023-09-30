cmake_minimum_required(VERSION 3.10)

# Your project name
project(MyRoboticsProject)

# Add GoogleTest
enable_testing()
find_package(GTest CONFIG REQUIRED)

# Your existing settings
add_executable(MyRoboticsProject src/main.cpp src/body.cpp)
target_link_libraries(MyRoboticsProject GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main)

# Add your test cpp file
add_executable(runUnitTests tests/body_test.cpp)

# Link test executable against GoogleTest
target_link_libraries(runUnitTests GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main)
add_test(runUnitTests runUnitTests)
