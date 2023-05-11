#!/bin/bash
set -e

source "/opt/ros/noetic/setup.bash"
cd /app/
source "./devel/setup.bash"
exec "$@"