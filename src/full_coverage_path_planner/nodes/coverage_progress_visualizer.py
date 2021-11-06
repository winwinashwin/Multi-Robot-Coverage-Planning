#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from typing import Tuple

# Constants for more readable index lookup
X, Y, Z, W = 0, 1, 2, 3


class CoverageProgressNode(object):
    """The CoverageProgressNode keeps track of coverage progress.
    It does this by periodically looking up the position of the coverage disk in an occupancy grid.
    Cells within a radius from this position are 'covered'

    Cell values are interpreted in this way: Lower is covered, higher is less covered
    - 100: uncovered (initial value)
    - < 100: covered
    """

    DIRTY = 100

    def __init__(self):
        self.listener = tf.TransformListener()

        self._coverage_area: Tuple[float, float] or None = None

        self.coverage_resolution: float or None = None  # How big is a cell [m]

        # How well covered is a cell after it has been covered for 1 time step
        self.coverage_effectivity: float or None = None

        self.map_frame: str or None = None
        self.coverage_frame: str or None = None

        self.grid = self._initialize_map()

        self.grid_pub = rospy.Publisher("coverage_grid", OccupancyGrid, queue_size=1)

        self.reset_srv = rospy.Service("reset", Trigger, self.reset)

        self._rate = rospy.get_param("~rate", 0.5)
        self._update_timer = rospy.Timer(
            rospy.Duration(1.0 / self._rate), self._update_callback
        )

    def _initialize_map(self):
        # Initialize coverage matrix

        # Define parameters
        x = rospy.get_param(
            "~grid/width", 10.0
        )  # height of the area to cover, in x direction of the map
        y = rospy.get_param(
            "~grid/height", 10.0
        )  # width of the area to cover, in y direction of the map
        self._coverage_area = (x, y)

        self.coverage_radius_meters = rospy.get_param("~coverage/radius")

        self.coverage_resolution = rospy.get_param(
            "~grid/resolution", 0.05
        )  # How big is a cell [m]

        # How much covered is a cell after it has been covered for 1 time step
        self.coverage_effectivity = rospy.get_param("~coverage/effectivity", 5)

        self.map_frame = rospy.get_param("~map_frame")
        self.coverage_frame = rospy.get_param("~coverage_frame")

        self.coverage_radius_meters += (
            2 * self.coverage_resolution
        )  # Compensate for discretization
        self.coverage_radius_cells = int(
            self.coverage_radius_meters / self.coverage_resolution
        )

        grid = OccupancyGrid()
        grid.info.resolution = self.coverage_resolution

        grid.info.width = abs(int(self._coverage_area[X] / self.coverage_resolution))
        grid.info.height = abs(
            int(self._coverage_area[Y] / self.coverage_resolution)
        )

        grid.info.origin.position.x = rospy.get_param("~/grid/origin/x", -5.0)
        grid.info.origin.position.y = rospy.get_param("~/grid/origin/y", -5.0)
        grid.info.origin.orientation.w = 1

        # Initialize OccupancyGrid to have all cells DIRTY
        # NOTE: not working with grid.data directly. Numpy arrays are convenient but when passed to publisher
        # it throws serialization errors. Hence work with an array separately finally convert to native list before
        # passing to publisher
        self.grid_data = self.DIRTY * np.ones(
            grid.info.width * grid.info.height, dtype=np.int8
        )

        return grid

    def _update_callback(self, _event):
        # Get the position of point (0,0,0) the coverage_disk frame with respect to
        # the map frame (which can both be remapped if needed)

        try:
            (coverage_pos, rot) = self.listener.lookupTransform(
                self.map_frame, self.coverage_frame, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            return

        # Element of matrix corresponding to middle of coverage surface
        x_point = int(
            (coverage_pos[X] - self.grid.info.origin.position.x)
            / self.coverage_resolution
        )
        y_point = int(
            (coverage_pos[Y] - self.grid.info.origin.position.y)
            / self.coverage_resolution
        )

        # Initialize message
        self.grid.header = Header()
        self.grid.header.frame_id = self.map_frame

        # Loop over amount of cells covered in x (j) and y (i) direction
        for i in range(2 * self.coverage_radius_cells):
            for j in range(2 * self.coverage_radius_cells):

                x_index = j - self.coverage_radius_cells
                y_index = i - self.coverage_radius_cells

                array_index = (
                    x_point + x_index + self.grid.info.width * (y_point + y_index)
                )

                cell_in_coverage_circle = (
                    x_index ** 2 + y_index ** 2 < self.coverage_radius_cells ** 2
                )

                cell_in_grid = 0 <= x_point + x_index < abs(
                    int(self._coverage_area[X] / self.coverage_resolution)
                ) and 0 <= y_point + y_index < abs(
                    int(self._coverage_area[Y] / self.coverage_resolution)
                )

                if cell_in_coverage_circle and cell_in_grid:
                    self.grid_data[array_index] *= 1.0 - self.coverage_effectivity
        else:
            rospy.logdebug(
                "x_point %i y_point %i, x_meas %f, y_meas %f",
                x_point,
                y_point,
                coverage_pos[X],
                coverage_pos[Y],
            )

        self.grid.data = self.grid_data.tolist()
        # Convert to list to avoid serialization errors
        self.grid_pub.publish(self.grid)

    def reset(self, _srv_request):
        rospy.loginfo("Reset coverage progress and grid")
        self.grid = self._initialize_map()
        return True, "Reset coverage progress and grid"


if __name__ == "__main__":
    rospy.init_node("coverage_progress", anonymous=True)
    try:
        node = CoverageProgressNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
