# syntax=docker/dockerfile:1
# Multi-stage image for the robotics-ws workspace (ROS 2 Jazzy / Gazebo Harmonic).
#
#   docker build --target dev -t robotics-ws:dev .
#   docker run -it --rm --net=host --ipc=host robotics-ws:dev
#
# Stage 1 (deps) installs every rosdep dependency declared by the packages in
# src/ — this layer is cached, so day-to-day rebuilds only recompile changed
# code. Stage 2 (build) compiles the workspace. Stage 3 (dev) adds the lint
# toolchain and a sourced entrypoint — CI builds this target as a full
# compile check of every package.

FROM ros:jazzy-ros-base AS deps
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /ws
COPY src ./src
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

FROM deps AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM build AS dev
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY requirements-lint.txt ruff.toml .pylintrc docker-entrypoint.sh /tmp/ctx/
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-pip \
        python3-pytest \
    && pip3 install --break-system-packages \
        -r /tmp/ctx/requirements-lint.txt \
    && rm -rf /var/lib/apt/lists/* /tmp/ctx/requirements-lint.txt \
    && mv /tmp/ctx/ruff.toml /tmp/ctx/.pylintrc /ws/ \
    && mv /tmp/ctx/docker-entrypoint.sh /usr/local/bin/ \
    && chmod +x /usr/local/bin/docker-entrypoint.sh
# A sourced shell on entry: ros + workspace overlays
ENTRYPOINT ["docker-entrypoint.sh"]
CMD ["bash"]
