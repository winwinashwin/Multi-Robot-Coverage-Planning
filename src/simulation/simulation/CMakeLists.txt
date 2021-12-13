cmake_minimum_required(VERSION 3.0.2)
project(simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# No more annoying gazebo warnings
if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

find_package(catkin REQUIRED COMPONENTS
        gazebo_msgs
        nav_msgs
        roscpp
        roslint
        std_srvs
        std_msgs
        )
find_package(gazebo REQUIRED)
#catkin_lint: ignore_once link_directory
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES gazebo_2Dmap_plugin
        CATKIN_DEPENDS roscpp nav_msgs std_srvs gazebo_msgs std_msgs
        DEPENDS GAZEBO
)
roslint_cpp()
roslint_python()

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_2Dmap_plugin SHARED plugins/gazebo_2Dmap_plugin.cpp)
target_link_libraries(gazebo_2Dmap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_2Dmap_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gz_realtime_factor_node src/gz_realtime_factor_node.cpp)
target_link_libraries(gz_realtime_factor_node ${catkin_LIBRARIES})
add_dependencies(gazebo_2Dmap_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

catkin_install_python(PROGRAMS
        scripts/multi_robot_spawner.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS gazebo_2Dmap_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS gz_realtime_factor_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)

    # test all launch files
    roslaunch_add_file_check(launch)
    roslint_add_test()
endif ()