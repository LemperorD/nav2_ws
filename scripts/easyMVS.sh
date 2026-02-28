#!/bin/bash

LOG_DIR=~/nav2_ws/MVS_logs

mkdir -p "$LOG_DIR"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MVCAM_SDK_PATH/bin:$MVCAM_COMMON_RUNENV/64

export MV_LOG_PATH=$LOG_DIR

exec $MVCAM_SDK_PATH/bin/MVS -platform xcb "$@"