#!/bin/bash

#SBATCH --partition=normal
#SBATCH --job-name=GetResults
#SBATCH --output=cpu_report/r-cpu-test-%j.out
#SBATCH --error=cpu_report/r-cpu-test-%j.err
#SBATCH --time=4-23:59:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=8
#SBATCH --mem-per-cpu=4GB

SINGULARITY_BASE=/containers/dgx/UserContainers/
SINGULARITY_IMG=$SINGULARITY_BASE/ylu22/jackal-final.sif

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR=/tmp

module load singularity

echo "=== 作业信息 ==="
echo "作业ID: $SLURM_JOB_ID"
echo "节点名称: $SLURM_NODELIST"
echo "CPU类型: $(cat /proc/cpuinfo | grep 'model name' | head -1 | cut -d':' -f2)"
echo "CPU核心数: $SLURM_CPUS_PER_TASK"
echo "内存: $(free -h | grep Mem | awk '{print $2}')"
echo "开始时间: $(date)"
echo "================="

cd ..

ACTOR_ID=$1
MODEL_ID=$2
BASE_LINE=$3

./singularity_run.sh $SINGULARITY_IMG python3 td3/tester.py --buffer_path buffer/ --id $ACTOR_ID --test_id $MODEL_ID --baseline $BASE_LINE