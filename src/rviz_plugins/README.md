# Overview
This package contains a Qt RViz panel that allows users to display topics values dynamically.

Supported [built-in types](http://wiki.ros.org/msg#Fields) are:
- `bool`
- `duration`
- `float32`
- `float64`
- `int8`
- `int16`
- `int32`
- `int64`
- `string`
- `time`
- `uint8`
- `uint16`
- `uint32`
- `uint64`

:warning: Qt does not support displaying some types, these are casted:
- `int64` is casted to a `int32_t`
- `uint32` is casted to a `uint16_t`
- `uint64` is casted to a `uint16_t`

:warning: It is not possible to display values from inside custom messages.

# Screenshots
![Pick topics](documentation/pick_topics.png)

![Graphical user interface](documentation/gui.png)

![Settings](documentation/settings.png)

# Dependencies

## rosdep
Install, initialize and update [rosdep](https://wiki.ros.org/rosdep).

# Compiling
Create a catkin workspace and clone the project:

```bash
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone https://gitlab.com/InstitutMaupertuis/topics_rviz_plugin.git
cd ..
```

## Resolve ROS dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## Compile
```bash
catkin_make
```

# How to use
Launch RViz and add the panel via the menu: `Panels` > `Add new panel` > `topics_rviz_plugin` > `DisplayTopics`
