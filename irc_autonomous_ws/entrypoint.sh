#!/bin/bash

# This tells the shell to exit if any command returns a non-zero exit status
# Non-zero error is an indication of failure in Unix systems
set -e

# These two commands will source the overlay and underlay
# such that we can run our package
source /opt/ros/humble/setup.bash
source /app/irc_autonomous_ws/install/setup.bash

# This means that we are doing everything in this entrypoint.sh script,
# then in the same shell, we will run the command the user passes in on the command line.
# (if the user passes a command)
exec "$@"
