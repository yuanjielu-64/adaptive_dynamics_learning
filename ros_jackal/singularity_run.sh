#!/bin/bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

apptainer exec -i --nv -n --network=none -p -B `pwd`:/jackal_ws/src/ros_jackal ${1} /bin/bash /jackal_ws/src/ros_jackal/entrypoint.sh ${@:2}
