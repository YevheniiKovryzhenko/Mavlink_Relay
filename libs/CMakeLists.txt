#Libs folder

#This is a list of all the libraries actually used within the project:
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Mavlink/") #MAVLINK 
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/optitrack/") #OPTITRACK
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/serial_tools_lib/")