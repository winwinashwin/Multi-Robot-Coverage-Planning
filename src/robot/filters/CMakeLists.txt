cmake_minimum_required(VERSION 3.0.2)
project(filters)

find_package(catkin REQUIRED COMPONENTS
        rospy
        roslint
        )
roslint_python()

catkin_package()

catkin_install_python(PROGRAMS
        scripts/imu_frame_id_correction.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)

    # test all launch files
    roslaunch_add_file_check(launch)
    roslint_add_test()
endif ()