#!/bin/bash

#SBATCH --partition=gpuq
#SBATCH --qos=gpu
#SBATCH --job-name=single-gpu
#SBATCH --output=gpu_report/r-test-%j.out
#SBATCH --error=gpu_report/r-test-%j.err
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --gres=gpu:A100.40gb:1
#SBATCH --mem=500GB
#SBATCH --export=ALL
#SBATCH --time=0-12:00:00

set echo
umask 0027

nvidia-smi

SINGULARITY_BASE=/containers/dgx/UserContainers/
SINGULARITY_IMG=$SINGULARITY_BASE/ylu22/jackal-final.sif

module load singularity

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR=/tmp

cd ..

./singularity_run.sh $SINGULARITY_IMG python3 td3/train.py --buffer_path buffer/ --config_path configs/ --logging_path logging/ --config_file TD3_cluster --world_path jackal_helper/worlds/BARN/