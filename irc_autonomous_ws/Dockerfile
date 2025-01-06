# This is the image of ROS2 that is pulled (downloaded and included in the local image)
FROM osrf/ros:humble-desktop

# We are using bash
SHELL ["/bin/bash", "-c"]

# Here we specify the work directory inside of our container, everything will be placed inside /app
WORKDIR /app

# Here, we are copying "src" folder from our workspace folder to the newly created folder irc_autonomous_ws/src inside of our container.
# That is, we are creating "irc_autonomous_ws" folder in the container and another subfolder inside of that folder called "src".
# The path in the container is /app/irc_autonomous_ws/src
COPY src irc_autonomous_ws/src

# Here we are starting from /app and moving to /app/irc_autonomous_ws and we are sourcing the
# base environment that is now inside of our container and we are building the package
RUN cd irc_autonomous_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

# Here, we are copying another file from our local computer to the root folder "/"
# inside of our container.
# See entrypoint.sh file for more details
COPY entrypoint.sh /

# Here we are specifying the entry point file
ENTRYPOINT ["/entrypoint.sh"]

# At runtime, the command "bash" will be executed and this shell will be started
CMD ["bash"]
