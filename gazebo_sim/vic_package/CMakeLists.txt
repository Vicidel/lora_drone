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
add_executable(vic_takeoff
	src/vic_takeoff.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(vic_takeoff
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# create executable name
add_executable(vic_mission
	src/vic_mission.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(vic_mission
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# create executable name
add_executable(vic_mission_3drones
	src/vic_mission_3drones.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(vic_mission_3drones
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# create executable name
add_executable(vic_coordinates
	src/vic_coordinates.cpp
	src/server_app.cpp
	src/cJSON.c
)

# used librairies
target_link_libraries(vic_coordinates
	${catkin_LIBRARIES}
	${CURL_LIBRARY}
	${CMAKE_LIBRARY_PATH}
) 

# for curl
find_package(CURL)
if(CURL_FOUND)
	include_directories(${CURL_INCLUDE_DIRS})
	target_link_libraries(vic_mission ${CURL_LIBRARIES})
	target_link_libraries(vic_mission_3drones ${CURL_LIBRARIES})
else()
	MESSAGE(FATAL_ERROR "LibCurl is not found in your system.")
endif(CURL_FOUND)