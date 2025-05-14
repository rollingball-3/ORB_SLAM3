#!/bin/bash
# 终止所有可能的卡住的mono_inertial_euroc进程
pkill -f mono_inertial_euroc

# 创建日志目录（如果不存在）
LOG_DIR="/data/qiufangzhou/robot/ORB_SLAM3/logs"
mkdir -p $LOG_DIR

# 生成带时间戳的日志文件名
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_FILE="$LOG_DIR/mono_inertial_euroc_$TIMESTAMP.log"

echo "Running mono_inertial_euroc. Output will be saved to: $LOG_FILE"

# 使用绝对路径确保文件能被找到，并将输出重定向到日志文件
/data/qiufangzhou/robot/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc \
    /data/qiufangzhou/robot/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /data/qiufangzhou/robot/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml \
    /data/qiufangzhou/robot/ORB_SLAM3 \
    /data/qiufangzhou/robot/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt \
    2>&1 | tee $LOG_FILE

echo "Execution completed. Log saved to: $LOG_FILE"
