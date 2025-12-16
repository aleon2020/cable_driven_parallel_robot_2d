#!/bin/bash

case "$1" in

  # NUMBER 1 (0.4 x 0.4 SURFACE AREA)
  small)
    POSES="
      {header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.5, z: 0.0}}},
      {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.7, z: 0.0}}},
      {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.3, z: 0.0}}}
    "
    ;;

  # NUMBER 1 (0.6 x 0.6 SURFACE AREA)
  medium)
    POSES="
      {header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: 0.5, z: 0.0}}},
      {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.8, z: 0.0}}},
      {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.2, z: 0.0}}}
    "
    ;;

  # NUMBER 1 (0.8 x 0.8 SURFACE AREA)
  large)
    POSES="
      {header: {frame_id: 'world'}, pose: {position: {x: 0.1, y: 0.5, z: 0.0}}},
      {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.9, z: 0.0}}},
      {header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.1, z: 0.0}}}
    "
    ;;

  # ERROR MESSAGE 
  *)
    echo "usage: $0 [ small | medium | large ]"
    exit 1
    ;;

esac

# EXECUTED COMMAND
ros2 topic pub --once /cdpr nav_msgs/msg/Path \
"{header: {frame_id: 'world'}, poses: [ $POSES ]}"
