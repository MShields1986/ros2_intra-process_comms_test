ARG ROS_DISTRO=lyrical

FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# RMW selector: one of cyclonedds, fastdds, zenoh
ARG RMW_IMPL=cyclonedds
ENV RMW_IMPL=${RMW_IMPL}

# Install the selected RMW and export RMW_IMPLEMENTATION via /etc/profile.d
COPY docker/rmw/${RMW_IMPL}.sh /tmp/install_rmw.sh
RUN chmod +x /tmp/install_rmw.sh \
 && /tmp/install_rmw.sh \
 && rm -rf /var/lib/apt/lists/*

# Build the workspace
COPY src /ros2_ws/src
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
 && rm -rf /var/lib/apt/lists/* \
 && JOBS=$(( $(nproc) / 2 > 0 ? $(nproc) / 2 : 1 )) \
 && MAKEFLAGS="-j${JOBS}" colcon build --symlink-install \
        --parallel-workers "${JOBS}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash'       >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash'       >> /ros_entrypoint.sh

# Ensure RMW_IMPLEMENTATION is exported in interactive shells and entrypoint.
RUN cat /etc/profile.d/ipc_rmw.sh >> /root/.bashrc \
 && sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'source /etc/profile.d/ipc_rmw.sh' >> /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]

ENV DEBIAN_FRONTEND=dialog
