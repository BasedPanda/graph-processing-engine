cmake_minimum_required(VERSION 3.14)

# Find Google Test
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Collect all test source files
set(TEST_SOURCES
    test_main.cpp
    graph_test.cpp
    algorithm_test.cpp
    engine_test.cpp
)

# Create test executable
add_executable(graph_engine_tests ${TEST_SOURCES})

# Link against Google Test and our library
target_link_libraries(graph_engine_tests
    PRIVATE
    graph_engine_lib  # This should be the name of your main library target
    GTest::gtest
    GTest::gtest_main
)

# Include directories
target_include_directories(graph_engine_tests
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

# Add compiler flags for tests
target_compile_features(graph_engine_tests PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(graph_engine_tests PRIVATE /W4)
else()
    target_compile_options(graph_engine_tests PRIVATE -Wall -Wextra)
endif()

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(graph_engine_tests)