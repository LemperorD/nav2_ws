#!/bin/bash
cd ~/nav2_ws

ROOT_PATH=${MVCAM_SDK_PATH}/bin

export LD_LIBRARY_PATH=${ROOT_PATH}:${MVCAM_COMMON_RUNENV}/64

if [ "$EUID" -ne 0 ]; then
  echo "not root user" 
else
  echo "Run as root"
  chmod -R 777 ${ROOT_PATH}/Cfg
  chmod -R 777 ${ROOT_PATH}/Temp
fi

exec ${ROOT_PATH}/MVS -platform xcb $1 $2
