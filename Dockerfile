# Lightweight custom image for this workspace based on the Jazzy ROS2 image
# Installs python3-serial and nano so the container is ready-to-run.

FROM ros:jazzy-perception

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-serial \
       nano \
    && rm -rf /var/lib/apt/lists/*

# Workspace mount point (user mounts their workspace at runtime)
RUN mkdir -p /ros2_ws
WORKDIR /ros2_ws

# Keep a shell as the default entrypoint for interactive use
CMD ["/bin/bash"]
