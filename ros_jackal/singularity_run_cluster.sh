#!/bin/bash
export PATH=$PATH:/lusr/opt/apptainer/bin
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
# export IMAGE_PATH=/scratch/cluster/zifan/ros_jackal_image.sif

apptainer exec -i --nv -n --network=none -p -B /var/condor:/var/condor -B `pwd`:/jackal_ws/src/ros_jackal -B ${BUFFER_PATH}:${BUFFER_PATH} ${IMAGE_PATH} /bin/bash /jackal_ws/src/ros_jackal/entrypoint.sh ${@:1}