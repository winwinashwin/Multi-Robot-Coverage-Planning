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


def main():
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
        # Not using poses[1] because the points maybe so close and orientation won't be accurate. 15 was found to be a
        # good number from trial and error. This might change with the up-sampling parameter value during path
        # computation.
        second = path.poses[15].pose  # type: Pose

        x1, y1 = first.position.x, first.position.y
        x2, y2 = second.position.x, second.position.y
        yaw = atan2(y2 - y1, x2 - x1)
        cli_args = [
            pkg,
            src,
            f"id:={robot_id}",
            f"x_pos:={x1}",
            f"y_pos:={y1}",
            f"yaw:={yaw}",
        ]
        rospy.loginfo(f"Spawning {robot_id} with x={x1}, y={y1}, yaw={yaw}")
        rospy.loginfo(f"{(x1, y1)}, {(x2, y2)}")
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_args = cli_args[2:]
        launch_files.append((roslaunch_file, roslaunch_args))

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()
    parent.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
