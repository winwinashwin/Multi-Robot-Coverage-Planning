cmake_minimum_required(VERSION 3.0.2)
project(slam)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)

    # test all launch files
    roslaunch_add_file_check(launch)
endif ()