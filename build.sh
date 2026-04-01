#!/bin/bash

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash

if [ $# -eq 0 ]; then
    echo "执行全部构建"
    colcon build
elif [ "$1" = "symlink" ]; then
    echo "执行软链接构建 --symlink-install"
    colcon build --symlink-install
else
    PKG_NAME="$1"
    echo "指定功能包构建: $PKG_NAME"
    colcon build --packages-select "$PKG_NAME"
fi