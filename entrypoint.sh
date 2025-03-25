#!/bin/bash
set -e

if [ ! -f /root/catkin_ws/devel/setup.bash ]; then
    echo "Workspace não encontrado em /root/catkin_ws. Restaurando backup..."
    cp -r /root/catkin_ws_default/* /root/catkin_ws/
fi

exec "$@"
