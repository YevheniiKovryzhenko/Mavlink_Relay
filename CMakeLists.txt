cmake_minimum_required(VERSION 3.8)

project("Mavlink_Relay")

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Collect all source files for the main application
file(GLOB ALL_MAIN 
    "include/*.h" "include/*.hpp"
    "src/*.c" "src/*.cpp"
)

# Add subdirectory for external libraries
add_subdirectory(libs)

# Main executable
add_executable(Mavlink_Relay ${ALL_MAIN})

# Organize source files into groups for IDEs
source_group("ALL_FILES" FILES ${ALL_MAIN})

# Include directories for the main application
target_include_directories(Mavlink_Relay PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
)

# Link external libraries
target_link_libraries(Mavlink_Relay PUBLIC mavlink optitrack Serial_Tools_lib)

# Enable threading support
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(Mavlink_Relay PRIVATE Threads::Threads)

# -------------------------
# Multicast Tests Integration
# -------------------------

# Create a subdirectory for tests
add_subdirectory(tests)