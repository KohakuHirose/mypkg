# Health Monitoring Syste

[![test](https://github.com/KohakuHirose/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/KohakuHirose/mypkg/actions/workflows/test.yml)
[![license](https://img.shields.io/badge/license-BSD--3--Clause-green?style=flat)](https://github.com/KohakuHirose/mypkg?tab=BSD-3-Clause-1-ov-file)

## 概要

このプロジェクトは、健康データ（心拍数と体温）のシミュレーションを行い、異常値を検出した場合にアラートを発行する ROS 2 ノード群を提供します。  
以下の機能を含みます:
- **データ生成**: 心拍数と体温を一定間隔で生成。
- **異常値の監視**: 範囲外の値を検出し、アラートをパブリッシュ。
- **ローンチファイル**: すべてのノードを一括起動。

---

## ノードの説明

### 1. **パブリッシャーノード: HealthDataPublisher**

- **役割**:
  健康データ（心拍数と体温）をシミュレーションし、以下のトピックにパブリッシュします。
  - `heart_rate` (心拍数)
  - `body_temperature` (体温)

- **実装のポイント**:
  - 正常値をランダムに生成。
  - **10% の確率で異常値を生成**:
    - 心拍数: 40.0 ～ 120.0 bpm
    - 体温: 35.0 ～ 39.0 °C
  - タイマーで 1 秒ごとにデータを生成。

### 2. **サブスクライバーノード: HealthMonitor**

- **役割**:
  健康データを購読し、異常値を検出した場合にアラートを発行します。
  - アラートは `health_alerts` トピックにパブリッシュされます。

- **実装のポイント**:
  - **正常値の範囲**:
    - 心拍数: 60.0 ～ 100.0 bpm
    - 体温: 36.0 ～ 37.5 °C
  - 範囲外の値が検出された場合:
    - アラートメッセージをパブリッシュ。
    - ログに警告を記録。

### 3. **ローンチファイル: health_monitoring.launch.py**

- **役割**:
  パブリッシャーノード (`health_data_publisher`) とサブスクライバーノード (`health_monitor`) を同時に起動。

---

## トピック

- **`heart_rate`**:
  - 型: `std_msgs.msg.Float32`
  - 心拍数を送信。

- **`body_temperature`**:
  - 型: `std_msgs.msg.Float32`
  - 体温を送信。

- **`health_alerts`**:
  - 型: `std_msgs.msg.String`
  - 異常値を検出した際のアラート。

---

## 実行方法

### セットアップ

1. ワークスペースを作成
``` bash
mkdir -p ros2_ws/src
cd ~/ros2_ws/src/
```

2. リポジトリをクローン
``` bash
git clone https://github.com/KohakuHirose/mypkg.git
```

3. ワークスペースをビルド
``` bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 起動
1. ローンチファイルの実行
``` bash
ros2 launch mypkg health_monitoring.launch.py
```

2. アラートの監視：別のターミナルで以下を実行してアラートを確認
``` bash
ros2 topic echo /health_alerts
```

## 実行可能なソフトウェア
- **Python**
	- テスト済みバージョン: 3.7~3.10

## テスト環境
- **Ubuntu** 22.04 LTS

## ライセンス
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．
- このパッケージのコードの一部は，下記のスライド（CC-BY-SA 4.0 by Ryuichi Ueda）のものを，本人の許可を得て自身の著作としたものです．
    - [ryuichiueda/slides_marp/robosys2024](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2024)

 © 2025 Kohaku Hirose


