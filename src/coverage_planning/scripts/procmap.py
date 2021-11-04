#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import rospy
from pathlib import Path


rospy.init_node("procmap", anonymous=True)

config = {
    "map": rospy.get_param("~map"),
    "cardinality_gain": rospy.get_param("~cardinality_gain"),
    "time_gain": rospy.get_param("~time_gain"),
    "visualize": rospy.get_param("~visualize"),
}

# read map metadata
with open(config["map"]) as fp:
    metadata = yaml.safe_load(fp.read())

map_img = Path(config["map"]).parent.joinpath(metadata["image"])
assert map_img.exists()
src = cv2.imread(str(map_img), cv2.IMREAD_GRAYSCALE)

_, gray = cv2.threshold(
    src, (1 - metadata["free_thresh"]) * 255, 255, cv2.THRESH_BINARY
)
contours, _ = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

navigable = max(contours, key=lambda c: cv2.contourArea(c))
area = cv2.contourArea(navigable)

if config['visualize']:
    (x, y, w, h) = cv2.boundingRect(navigable)
    cv2.rectangle(gray, (x, y), (x+w, y+h), 255, 1)
    cv2.imshow('image', gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Penalty for number of robots
a = config["cardinality_gain"]
# Penalty for time
b = config["time_gain"]

#
# minimise objective a*N + b*(area/(resolution^2))/N
#

b_eff = b * area / np.power(metadata["resolution"], 2)
n_agents = np.ceil(np.sqrt(b_eff / a / 1e6))
# Minimum 2 agents are required
n_agents = np.clip(n_agents, a_min=2, a_max=np.inf)

rospy.loginfo(f"Computed number of agents: {n_agents}")
