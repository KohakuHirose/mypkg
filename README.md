# Health Monitoring Syste

[![test](https://github.com/KohakuHirose/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/KohakuHirose/mypkg/actions/workflows/test.yml)
[![license](https://img.shields.io/badge/license-BSD--3--Clause-green?style=flat)](https://github.com/KohakuHirose/mypkg?tab=BSD-3-Clause-1-ov-file)

## 概要

このノードシステムは、ROS2を用いて健康データ（心拍数と体温）のシミュレーションを行い、異常値を検出した場合にアラートをパブリッシュ（送信）するパッケージです。

- 機能
  - **データ生成**：心拍数と体温を一定間隔で生成。
  - **異常値の監視**：範囲外の値を検出し、アラートをパブリッシュ。
  - **ローンチファイル**：すべてのノードを一括起動。

## ノードの説明

### 1. **パブリッシャーノード：health_data_publisher.py**

- **役割**
  - 健康データ（心拍数と体温）をシミュレーションし、以下のトピックにパブリッシュします。
    - `heart_rate` (心拍数)
    - `body_temperature` (体温)

- **実装のポイント**
  - 正常値をランダムに生成。
  - **30% の確率で異常値を生成**:
    - 心拍数: 40.0 ～ 120.0 bpm
    - 体温: 35.0 ～ 39.0 °C
  - タイマーで 1 秒ごとにデータを生成。

### 2. **サブスクライバーノード：health_monitor.py**

- **役割**
  - 健康データを受信し、異常値を検出した場合にアラートを出力します。
    - アラートは `health_alerts` トピックにパブリッシュされます。

- **実装のポイント**
  - **正常値の範囲**
    - 心拍数: 60.0 ～ 100.0 bpm
    - 体温: 36.0 ～ 37.5 °C
  - 範囲外の値が検出された場合:
    - アラートメッセージをパブリッシュ。
    - ログに警告を記録。

## 実行方法

### 起動
1. ローンチファイルの実行
```bash
ros2 launch mypkg health_monitoring.launch.py
```

実行結果：例1
```
$ ros2 launch mypkg health_monitoring.launch.py
[INFO] [launch]: All log files can be found below /home/hirose/.ros/log/2025-01-03-19-19-39-404073-DESKTOP-N1MNJB1-7651
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [health_data_publisher-1]: process started with pid [7653]
[INFO] [health_monitor-2]: process started with pid [7655]
[health_data_publisher-1] [INFO] [1735899580.895537398] [health_data_publisher]: Published Heart Rate: 64.42, Body Temperature: 36.25
[health_data_publisher-1] [INFO] [1735899581.881114094] [health_data_publisher]: Published Heart Rate: 79.18, Body Temperature: 36.86
[health_data_publisher-1] [INFO] [1735899582.881475566] [health_data_publisher]: Published Heart Rate: 80.82, Body Temperature: 36.22
[health_data_publisher-1] [INFO] [1735899583.882422697] [health_data_publisher]: Published Heart Rate: 71.66, Body Temperature: 36.50
[health_data_publisher-1] [INFO] [1735899584.882369431] [health_data_publisher]: Published Heart Rate: 87.89, Body Temperature: 36.09
[health_data_publisher-1] [INFO] [1735899585.882449383] [health_data_publisher]: Published Heart Rate: 88.71, Body Temperature: 35.02
[health_monitor-2] [WARN] [1735899585.899211087] [health_monitor]: Abnormal Body Temperature Detected: 35.02 °C!
[health_data_publisher-1] [INFO] [1735899586.880927421] [health_data_publisher]: Published Heart Rate: 75.85, Body Temperature: 36.73
```

2. アラートの監視：別のターミナルで以下を実行してアラートを確認
```bash
ros2 topic echo /health_alerts
```

実行結果：例2
```
$ ros2 topic echo /health_alerts
data: 'Abnormal Body Temperature Detected: 35.56 °C!'
---
data: 'Abnormal Heart Rate Detected: 52.34 bpm!'
---
data: 'Abnormal Heart Rate Detected: 103.44 bpm!'
---
data: 'Abnormal Body Temperature Detected: 37.68 °C!'
---
data: 'Abnormal Heart Rate Detected: 105.72 bpm!'
---
data: 'Abnormal Body Temperature Detected: 35.28 °C!'
---
data: 'Abnormal Body Temperature Detected: 35.01 °C!'
---
data: 'Abnormal Body Temperature Detected: 35.28 °C!'
---
```

## 実行可能なソフトウェア
- Python 
    - テスト済みバージョン：3.7~3.10
- ROS2 Foxy

## テスト環境
- Ubuntu 22.04 LTS
- Docker ryuichiueda/ubuntu22.04-ros2:latest

## ライセンス
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．
- このパッケージのコードの一部は，下記のスライド（CC-BY-SA 4.0 by Ryuichi Ueda）のものを，本人の許可を得て自身の著作としたものです．
    - [ryuichiueda/slides_marp/robosys2024](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2024)

 © 2025 Kohaku Hirose


