#!/usr/bin/env python3

"""Merges coverage maps of each robot into a global coverage map.

robot_0/coverage_grid + robot_1/coverage_grid + ... + robot_n/coverage_grid ==> coverage_grid_merged

Assumptions:
All individual coverage grids have same width, height, resolution and origin and is of the same frame.
If any of these changes, consider using `multirobot_map_merge` pkg instead.

`multirobot_map_merge` had some drift in the merged map even though the dimensions and resolution
 of each individual map was the same. Hence this simple node.
"""

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
import message_filters
from std_msgs.msg import UInt8, Float64
from typing import cast


def resolve_topic(map_topic: str, robot_ns: str, robot_id: int):
    map_topic = map_topic.lstrip("/")
    robot_ns = robot_ns.rstrip("/")
    return f"{robot_ns}_{robot_id}/{map_topic}"


class MultiMapMergerNode:
    def __init__(self):
        self._robot_map_topic = rospy.get_param("~robot_map_topic", "map")
        self._robot_ns = rospy.get_param("~robot_namespace", "")
        self._merged_map_topic = rospy.get_param("~merged_map_topic", "map")

        n_agents = cast(
            UInt8, rospy.wait_for_message("number_of_agents", UInt8)
        ).data

        self._subs = [
            message_filters.Subscriber(
                resolve_topic(self._robot_map_topic, self._robot_ns, robot_id),
                OccupancyGrid,
            )
            for robot_id in range(n_agents)
        ]

        ts = message_filters.TimeSynchronizer(self._subs, 10)
        ts.registerCallback(self.merge_callback)

        self._merged_pub = rospy.Publisher(
            self._merged_map_topic, OccupancyGrid, queue_size=10
        )
        self._cov_progress_pub = rospy.Publisher(
            "coverage_progress", Float64, queue_size=10
        )

    def merge_callback(self, *grids):
        size = grids[0].info.width * grids[0].info.height

        merged = np.zeros(size, dtype=np.int)

        for g in grids:
            merged = np.bitwise_or(np.array(g.data, dtype=np.int), merged)

        msg = OccupancyGrid()
        msg.header.frame_id = grids[0].header.frame_id
        msg.header.stamp = rospy.Time().now()
        msg.info = grids[0].info
        msg.data = merged.tolist()

        counts = dict(zip(*np.unique(merged, return_counts=True)))
        progress = 100 * counts[75] / (counts[75] + counts[0])
        self._merged_pub.publish(msg)
        self._cov_progress_pub.publish(progress)


def main():
    rospy.init_node("map_merge", anonymous=True)
    _node = MultiMapMergerNode()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
