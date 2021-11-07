# Multi-Robot Coverage Planning

**Developed for : AIITRA Robotics Challenge 2021**

## Description

Software stack for cooperative multi-robot coverage.

*NOTE: Gazebo physics engine in a multi-robot simulation setting is really CPU intensive.*

### Features

* Solution for autonomous Multi-Robot SLAM using Frontier based exploration
* Optimizer for finding optimal number of agents required to cover an occupancy grid
* Coverage path planner based on Boustrophedon Cellular Decomposition
* PID path tracking 

## Resolving dependencies

```bash
sudo rosdep install -i -y --from-paths ./src 
```

## Usage

### Multi-Robot Coverage

```bash
roslaunch simulation multi_robot.launch map:=map2_small
roslaunch full_coverage_path_planner cover_map.launch map:=map2_small
```
NOTE: **Make sure to unpause physics!**

### Multi-Robot SLAM

```bash
# todo(ashwin)
```

## Reference

```
@article{DBLP:journals/corr/abs-1806-03581,
  author    = {Anirudh Topiwala and
               Pranav Inani and
               Abhishek Kathpal},
  title     = {Frontier Based Exploration for Autonomous Robot},
  journal   = {CoRR},
  volume    = {abs/1806.03581},
  year      = {2018},
  url       = {http://arxiv.org/abs/1806.03581},
  eprinttype = {arXiv},
  eprint    = {1806.03581},
  timestamp = {Mon, 13 Aug 2018 16:46:39 +0200},
  biburl    = {https://dblp.org/rec/journals/corr/abs-1806-03581.bib},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}

@InProceedings{10.1007/978-1-4471-1273-0_32,
    author="Choset, Howie
    and Pignon, Philippe",
    editor="Zelinsky, Alexander",
    title="Coverage Path Planning: The Boustrophedon Cellular Decomposition",
    booktitle="Field and Service Robotics",
    year="1998",
    publisher="Springer London",
    address="London",
    pages="203--209",
    abstract="Coverage path planning is the determination of a path that a robot must take in order to pass over each point in an environment. Applications include vacuuming, floor scrubbing, and inspection. We developed the boustrophedon cellular decomposition, which is an exact cellular decomposition approach, for the purposes of coverage. Each cell in the boustrophedon is covered with simple back and forth motions. Once each cell is covered, then the entire environment is covered. Therefore, coverage is reduced to finding an exhaustive path through a graph which represents the adjacency relationships of the cells in the boustrophedon decomposition. This approach is provably complete and Experiments on a mobile robot validate this approach.",
    isbn="978-1-4471-1273-0"
}
```