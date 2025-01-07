#!/bin/bash
# SPDX-FileCopyrightText: 2025 Kohaku Hirose 　　　　　
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

# ワークスペースに移動してビルド
cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# ノードを起動してログを取得
timeout 10 ros2 launch mypkg health_monitoring.launch.py | tee - /tmp/mypkg.log

# ログからアラートメッセージを確認
cat /tmp/mypkg.log | grep 'WARN' | grep 'Abnormal'

# 終了ステータスの確認
if [ $? -eq 0 ]; then
    echo "Test Passed: Alerts detected in logs."
else
    echo "Test Failed: No alerts detected."
    exit 1
fi

