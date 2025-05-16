#!/bin/bash
# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 创建日志目录（如果不存在）, 使用相对于脚本的路径
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p $LOG_DIR

# 生成带时间戳的日志文件名
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_FILE="$LOG_DIR/orbbec_bgrd_inertial_$TIMESTAMP.log"


# 定义mask文件路径
MASK_PATH="${SCRIPT_DIR}/Examples/RGB-D/mask.png"

# 确认mask文件存在
if [ ! -f "$MASK_PATH" ]; then
    echo "Warning: Mask file not found at $MASK_PATH"
    echo "SLAM will run without mask."
    
    # 使用绝对路径确保文件能被找到，并将输出重定向到日志文件
    echo "Running orbbec_bgrd_inertial without mask. Output will be saved to: $LOG_FILE"
    /data/qiufangzhou/robot/ORB_SLAM3/Examples/RGB-D-Inertial/orbbec_bgrd_inertial \
        /data/public/datasets/tactile/anyskin_hand2/slam/orbbec.h5 \
        /data/qiufangzhou/robot/ORB_SLAM3/Vocabulary/ORBvoc.txt \
        /data/qiufangzhou/robot/ORB_SLAM3/Examples/RGB-D-Inertial/orbbec.yaml \
        > "$LOG_FILE" 2>&1
else
    # 带mask参数运行
    echo "Running orbbec_bgrd_inertial with mask: $MASK_PATH. Output will be saved to: $LOG_FILE"
    /data/qiufangzhou/robot/ORB_SLAM3/Examples/RGB-D-Inertial/orbbec_bgrd_inertial \
        /data/public/datasets/tactile/anyskin_hand2/slam/orbbec.h5 \
        /data/qiufangzhou/robot/ORB_SLAM3/Vocabulary/ORBvoc.txt \
        /data/qiufangzhou/robot/ORB_SLAM3/Examples/RGB-D-Inertial/orbbec.yaml \
        "$MASK_PATH" \
        > "$LOG_FILE" 2>&1
fi

echo "SLAM processing completed. Check log file at: $LOG_FILE"