cmake_minimum_required(VERSION 3.1)
project(frontier_exploration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        costmap_2d
        geometry_msgs
        map_msgs
        move_base_msgs
        nav_msgs
        roscpp
        roslint
        std_msgs
        std_srvs
        tf
        visualization_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        actionlib
        actionlib_msgs
        costmap_2d
        geometry_msgs
        map_msgs
        move_base_msgs
        nav_msgs
        roscpp
        std_msgs
        std_srvs
        tf
        visualization_msgs)

roslint_cpp()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        ${catkin_INCLUDE_DIRS}
        include)

add_executable(${PROJECT_NAME}_node
        src/costmap_client.cpp
        src/explore.cpp
        src/frontier_search.cpp
        src/node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)

    # test all launch files
    roslaunch_add_file_check(launch)
    roslint_add_test()
endif()
