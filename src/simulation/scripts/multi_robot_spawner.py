#!/usr/bin/env python3

import roslaunch.rlutil
import roslaunch.parent
import roslaunch
import rospy
from std_msgs.msg import UInt8
from typing import cast
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from math import atan2


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

rospy.init_node("multi_robot_spawner", anonymous=True)

n_agents = cast(UInt8, rospy.wait_for_message("number_of_agents", UInt8)).data

launch_files = []

pkg = "simulation"
src = "spawn.launch"
robot_namespace = rospy.get_param("~robot_namespace", "robot")

for robot_id in range(n_agents):
    path_topic = f"{robot_namespace}_{robot_id}/waypoints"
    path = cast(Path, rospy.wait_for_message(path_topic, Path))
    first = path.poses[0].pose  # type: Pose
    second = path.poses[1].pose  # type: Pose

    cli_args = [
        pkg,
        src,
        f"id:={robot_id}",
        f"x_pos:={first.position.x}",
        f"y_pos:={first.position.y}",
        f"yaw:={atan2(second.position.y-first.position.y,second.position.x-first.position.x)}",
    ]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[2:]
    launch_files.append((roslaunch_file, roslaunch_args))

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()
parent.spin()
