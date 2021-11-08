ARG FROM_IMAGE=osrf/ros:noetic-desktop-full
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG DEBIAN_FRONTEND=noninteractive

###########################################################################################################
FROM $FROM_IMAGE AS frozen_stage

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

###########################################################################################################
FROM frozen_stage as cache_stage

# copy source code
ARG OVERLAY_WS
COPY ./src $OVERLAY_WS/src

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name 'package.xml' | \
        xargs -I {} cp --parents -t /tmp/opt {} && \
    find ./ -name 'CATKIN_IGNORE' | \
        xargs -I {} cp --parents -t /tmp/opt {} || \
    true

###########################################################################################################
FROM frozen_stage as base_stage

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

# install overlay dependencies
COPY --from=cache_stage /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
        --from-paths src \
        --ignore-src \
    && rm -rf /var/lib/apt/lists/*

###########################################################################################################
FROM base_stage as build_stage

ARG OVERLAY_WS

# build overlay source
COPY --from=cache_stage $OVERLAY_WS/src ./src
ARG CATKIN_ARGS='-DCMAKE_BUILD_TYPE=Release'
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin_make $CATKIN_ARGS install

###########################################################################################################
FROM base_stage as prod_stage

ARG OVERLAY_WS

COPY --from=build_stage $OVERLAY_WS/install ./install

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
    '$isource "$OVERLAY_WS/install/setup.bash"' \
    /ros_entrypoint.sh
