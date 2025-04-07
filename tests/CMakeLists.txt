# Specify the C++ standard for tests
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find threading library for tests
find_package(Threads REQUIRED)

# Add multicast sender executable
add_executable(multicast_sender multicast_sender.cpp)
target_link_libraries(multicast_sender PRIVATE Threads::Threads)

# Add multicast receiver executable
add_executable(multicast_receiver multicast_receiver.cpp)
target_link_libraries(multicast_receiver PRIVATE Threads::Threads)