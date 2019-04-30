# base requirement for cmake
cmake_minimum_required(VERSION 3.5)

# package name
project(vic_package)


# for curl package
set(requiredlibs)
set(CURL_LIBRARY "-lcurl")
set(CMAKE_INCLUDE_PATH "/usr/include")
#set(CMAKE_LIBRARY_PATH "/usr/lib/x86_64-linux-gnu/libncurses.so")

# find packages for mavros
find_package(catkin REQUIRED COMPONENTS
	mavros
	roscpp
	rospy
	std_msgs
)



###################################
## catkin specific configuration ##
###################################

# create the package
catkin_package()



###########
## Build ##
###########

# location of header files
include_directories(
	${catkin_INCLUDE_DIRS}
	${CURL_INCLUDE_DIRS}
	${CMAKE_INCLUDE_PATH}
)

# create executable name
add_executable(offb_node
	src/offb_node.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(offb_node
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# create executable name
add_executable(offb_node_v2
	src/offb_node_v2.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(offb_node_v2
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# create executable name
add_executable(offb_node_simple
	src/offb_node_simple.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(offb_node_simple
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# create executable name
add_executable(offb_node_simple_v2
	src/offb_node_simple_v2.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(offb_node_simple_v2
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# for curl
find_package(CURL)
if(CURL_FOUND)
	include_directories(${CURL_INCLUDE_DIRS})
	target_link_libraries(offb_node ${CURL_LIBRARIES})
	target_link_libraries(offb_node_v2 ${CURL_LIBRARIES})
	target_link_libraries(offb_node_simple ${CURL_LIBRARIES})
else()
	MESSAGE(FATAL_ERROR "LibCurl is not found in your system.")
endif(CURL_FOUND)