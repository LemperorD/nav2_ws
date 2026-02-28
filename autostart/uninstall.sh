#!/bin/bash

SERVICE_NAME="nav_auto_launch.service"
TARGET_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Removing $SERVICE_NAME ..."

# 必须 root 权限
if [ "$EUID" -ne 0 ]; then
  echo "Please run with sudo."
  exit 1
fi

# 停止
systemctl stop "$SERVICE_NAME"

# 取消自启
systemctl disable "$SERVICE_NAME"

# 删除文件
rm -f "$TARGET_PATH"

# 重新加载
systemctl daemon-reload

echo "Service removed successfully."