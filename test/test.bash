#!/bin/bash -xv
# SPDX-FileCopyrightText: 2025 Kohaku Hirose 　　　　　
# SPDX-License-Identifier: BSD-3-Clause

# 使用するディレクトリを設定
dir=~
[ "$1" != "" ] && dir="$1"

# ワークスペースに移動し、ビルドする
cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# 背景でノードを実行
ros2 run mypkg health_node > /tmp/mypkg_node.log 2>&1 &
NODE_PID=$!

sleep 5

# メッセージを購読して異常値を検出
timeout 30 ros2 topic echo /health_alerts > /tmp/mypkg_alerts.log

# ノードを停止
kill $NODE_PID

# ログから特定のアラートメッセージを検索
cat /tmp/mypkg_alerts.log | grep -E 'Alert: Abnormal (Heart Rate|Body Temperature)'

# テスト結果の判定
if [ $? -eq 0 ]; then
    echo "Test Passed: Alerts detected in /health_alerts."
    exit 0
else
    echo "Test Failed: No alerts detected in /health_alerts."
    exit 1
fi

