#!/usr/bin/env python3

"""Evaluate overall coverage progress.

Subscribes to both original map and progressing coverage map.
Compares both and publishes current percentage coverage
"""

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
import numpy as np


class CoverageProgressEvaluatorNode:
    def __init__(self):
        self._static_map_topic = rospy.get_param("~static_map_topic")
        self._cov_map_topic = rospy.get_param("~coverage_map_topic")

        self._static_map = rospy.wait_for_message(
            self._static_map_topic, OccupancyGrid
        )
        self._map_subscription = rospy.Subscriber(
            self._cov_map_topic, OccupancyGrid, self.map_callback
        )

        self._progress_publisher = rospy.Publisher(
            "coverage_progress", Float64, queue_size=5
        )

    def map_callback(self, coverage_map: OccupancyGrid):
        static_grid = np.array(self._static_map.data)
        coverage_grid = np.array(coverage_map.data)
        try:
            assert static_grid.shape == coverage_grid.shape
        except AssertionError:
            rospy.logwarn(
                f"Grid shape mismatch! Static grid ({static_grid.shape}) & Coverage grid ({coverage_grid.shape})"
            )
            return

        counts = dict(
            zip(*np.unique(coverage_grid[static_grid != -1], return_counts=True))
        )
        progress = 100 * counts[75] / (counts[0] + counts[75])
        self._progress_publisher.publish(progress)


def main():
    rospy.init_node("eval_progress", anonymous=True)
    _ = CoverageProgressEvaluatorNode()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
