# Health Monitoring Syste

[![test](https://github.com/KohakuHirose/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/KohakuHirose/mypkg/actions/workflows/test.yml)
[![license](https://img.shields.io/badge/license-BSD--3--Clause-green?style=flat)](https://github.com/KohakuHirose/mypkg?tab=BSD-3-Clause-1-ov-file)

## 概要

このノードシステムは、ROS2を用いて健康データ（心拍数と体温）のシミュレーションを行い、その健康データをパブリッシュするパッケージです。

## ノードの説明

### **health_node**

健康データ（心拍数と体温）をシミュレーションし、以下のトピックにパブリッシュします。
- `heart_rate` (心拍数)
- `body_temperature` (体温)
- `health_alerts` (心拍数と体温、異常値の場合はアラート)

- **実装のポイント**
  - 正常値をランダムに生成。
    - 心拍数: 50.0 ～ 110.0 bpm
    - 体温: 35.0 ～ 38.5 °C
  - タイマーで 1 秒ごとにデータを生成。

## 実行方法

### 起動
health_nodeを実行
```bash
$ ros2 run mypkg health_node
```

別端末でトピックを確認
```bash
$ ros2 topic echo /heart_rate
```
```
data: 58.64283752441406
---
data: 60.33660125732422
---
data: 75.04821014404297
---
data: 104.03706359863281
---
data: 76.83824920654297
---
```

```bash
$ ros2 topic echo /body_temperature
```
```
data: 37.47539520263672
---
data: 37.76638412475586
---
data: 35.01969909667969
---
data: 37.65845489501953
---
data: 35.83195877075195
---
```

```bash
$ ros2 topic echo /health_alerts
```
実行例
```
data: 'Alert: Abnormal Heart Rate: 108.11 bpm!'
---
data: 'Alert: Abnormal Body Temperature: 38.48 °C!'
---
data: 'Normal: Heart Rate: 80.89 bpm, Body Temperature: 36.88 °C.'
---
data: 'Normal: Heart Rate: 68.46 bpm, Body Temperature: 36.04 °C.'
---
data: 'Normal: Heart Rate: 97.68 bpm, Body Temperature: 36.46 °C.'
---
data: 'Alert: Abnormal Heart Rate: 53.40 bpm!'
---
```

## 実行可能なソフトウェア
- Python 
    - テスト済みバージョン：3.7~3.10
- ROS2 Foxy

## テスト環境
- Ubuntu 22.04 LTS

テストに使用したコンテナ
- ryuichiueda/ubuntu22.04-ros2:latest:

## ライセンス
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．
- このパッケージのコードの一部は，下記のスライド（CC-BY-SA 4.0 by Ryuichi Ueda）のものを，本人の許可を得て自身の著作としたものです．
    - [ryuichiueda/slides_marp/robosys2024](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2024)

 © 2025 Kohaku Hirose


