#!/bin/bash

SERVICE_NAME="nav_auto_launch.service"
TARGET_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Installing $SERVICE_NAME ..."

# 必须 root 权限
if [ "$EUID" -ne 0 ]; then
  echo "Please run with sudo."
  exit 1
fi

# 检查当前目录是否存在 service 文件
if [ ! -f "$SERVICE_NAME" ]; then
  echo "$SERVICE_NAME not found in current directory."
  exit 1
fi

# 复制
cp "$SERVICE_NAME" "$TARGET_PATH"

# 重新加载
systemctl daemon-reload

# 开机自启
systemctl enable "$SERVICE_NAME"

# 立即启动
systemctl start "$SERVICE_NAME"

echo "Installation complete."
systemctl status "$SERVICE_NAME" --no-pager