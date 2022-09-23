cmake_minimum_required (VERSION 3.8)

project ("Mavlink_Relay")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#making all headers and sources global and attachin to Cmake variable "ALL_MAIN"
file(GLOB ALL_MAIN 
"include/*.h" "include/*.hpp"
"src/*.c" "src/*.cpp")

add_subdirectory(libs) #libs/ has all of the required libraries 

add_executable(Mavlink_Relay ${ALL_MAIN}) #this tells the compiler what to compile (ALL_MAIN files) and calls them executables.

# at this point we can compile all the sources and headers individually, but they don't see 
# each other, so linking will fail. To fix this, add all the folders to the executable 
# tree (called using variable "Main") so linker can search over it for all #inludes:
target_include_directories(Mavlink_Relay PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(Mavlink_Relay PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/")


# Link all the outside libraries used in this project (from libs folder):
target_link_libraries(Mavlink_Relay PUBLIC mavlink)
target_link_libraries(Mavlink_Relay PUBLIC optitrack)

# This forces to use threads library:
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(Mavlink_Relay PRIVATE Threads::Threads) #just including pthread.h is not enough