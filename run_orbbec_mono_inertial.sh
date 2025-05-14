#!/bin/bash
# 终止所有可能的卡住的mono_inertial_orbbec进程
pkill -f mono_inertial_orbbec

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 创建日志目录（如果不存在）, 使用相对于脚本的路径
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p $LOG_DIR

# 生成带时间戳的日志文件名
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_FILE="$LOG_DIR/mono_inertial_orbbec_$TIMESTAMP.log"

echo "Running mono_inertial_orbbec. Output will be saved to: $LOG_FILE"

# 使用绝对路径确保文件能被找到，并将输出重定向到日志文件
/data/qiufangzhou/robot/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_orbbec \
    /data/public/datasets/tactile/anyskin_hand2/slam/orbbec.h5 \
    /data/qiufangzhou/robot/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /data/qiufangzhou/robot/ORB_SLAM3/Examples/Monocular-Inertial/orbbec.yaml \
    2>&1 | tee $LOG_FILE

echo "Execution completed. Log saved to: $LOG_FILE"


