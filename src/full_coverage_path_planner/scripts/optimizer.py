#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from pathlib import Path
import yaml
from std_msgs.msg import UInt8
from nav_msgs.msg import OccupancyGrid


class BaseOptimizer(object):
    CARDINALITY_GAIN = None
    TIME_GAIN = None

    def __init__(self):
        self.pub = rospy.Publisher(
            "number_of_agents", UInt8, queue_size=10, latch=True
        )

    def load_params(self):
        raise NotImplementedError

    def load_metadata(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError

    @classmethod
    def solve(cls, navigable_area, resolution):
        rospy.loginfo(f"Using CARDINALITY_GAIN: {cls.CARDINALITY_GAIN}")
        rospy.loginfo(f"Using TIME_GAIN: {cls.TIME_GAIN}")

        navigable_area *= np.power(resolution, 2)
        n_agents = np.ceil(
            np.sqrt(cls.TIME_GAIN * navigable_area / cls.CARDINALITY_GAIN)
        )
        # Minimum 2 agents are required
        n_agents = np.clip(n_agents, a_min=2, a_max=np.inf).astype(dtype=np.uint8)

        rospy.loginfo(f"Solved. N_AGENTS: {n_agents}")
        return n_agents
        # return 1


class OptimizerFromImage(BaseOptimizer):
    def __init__(self):
        super(OptimizerFromImage, self).__init__()

        self._map_file = None
        self._metadata = None

        self.load_params()
        self.load_metadata()

    def load_params(self):
        OptimizerFromImage.CARDINALITY_GAIN = rospy.get_param("~cardinality_gain")
        OptimizerFromImage.TIME_GAIN = rospy.get_param("~time_gain")

        self._map_file = rospy.get_param("~map_file")
        rospy.loginfo(f"Optimizing from image: {self._map_file}")

    def load_metadata(self):
        with open(self._map_file) as fp:
            self._metadata = yaml.safe_load(fp.read())

    def run(self):
        map_img = Path(self._map_file).parent.joinpath(self._metadata["image"])
        assert map_img.exists()
        src = cv2.imread(str(map_img), cv2.IMREAD_GRAYSCALE)

        _, gray = cv2.threshold(
            src, (1 - self._metadata["free_thresh"]) * 255, 255, cv2.THRESH_BINARY
        )
        contours, _ = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        navigable = max(contours, key=lambda c: cv2.contourArea(c))
        area = cv2.contourArea(navigable)

        n_agents = OptimizerFromImage.solve(area, self._metadata["resolution"])
        self.pub.publish(n_agents)

        (x, y, w, h) = cv2.boundingRect(navigable)
        cv2.rectangle(gray, (x, y), (x + w, y + h), 255, 1)
        cv2.imwrite("/tmp/optimizer_navigable.png", gray)


class OptimizerFromOccupancyGrid(BaseOptimizer):
    def __init__(self):
        super(OptimizerFromOccupancyGrid, self).__init__()

        self.load_params()
        self.load_metadata()

    def load_params(self):
        OptimizerFromOccupancyGrid.CARDINALITY_GAIN = rospy.get_param(
            "~cardinality_gain"
        )
        OptimizerFromOccupancyGrid.TIME_GAIN = rospy.get_param("~time_gain")

        rospy.loginfo("Optimizing from occupancy grid")

    def load_metadata(self):
        # We will get metadata from the `map` topic itself, not need to load separately
        pass

    def run(self):
        rospy.loginfo("Waiting for map to become available")
        occupancy_grid = rospy.wait_for_message("map", OccupancyGrid)
        rospy.loginfo("Received map")
        area = sum(0 <= x <= 25 for x in occupancy_grid.data)
        n_agents = OptimizerFromOccupancyGrid.solve(
            area, occupancy_grid.info.resolution
        )
        self.pub.publish(n_agents)


def main():
    rospy.init_node("optimizer", anonymous=True)
    if rospy.get_param("~from_map_file"):
        node = OptimizerFromImage()
    else:
        node = OptimizerFromOccupancyGrid()
    node.run()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
