# Expo RoboMaster Controller

M5StickC Plus2を使用したRoboMasterモーター制御システム。CANバス通信とPID制御により、外部からのUDPコマンドでモーターの回転速度を制御します。

## 機能

- **CAN通信**: RoboMasterモーターとの1Mbps CAN通信
- **PID制御**: 目標RPMに対する精密な速度制御
- **UDP制御**: WiFi経由での外部コマンド受信
- **IMUデータ**: 加速度・ジャイロセンサーデータの取得
- **リアルタイム監視**: JSON形式でのセンサーデータ送信

## ハードウェア要件

- **M5StickC Plus2**
- **RoboMasterモーター** (M3508またはM2006)
- **CANトランシーバー** (M5StickC Plus2内蔵)

### 配線

```
M5StickC Plus2:
- GPIO32: CAN TX
- GPIO33: CAN RX
```

## ソフトウェア要件

### Arduino IDE設定

1. **ESP32ボードパッケージ**: バージョン3.2.1
   ```
   https://dl.espressif.com/dl/package_esp32_index.json
   ```

2. **必要なライブラリ**:
   - M5StickCPlus2 (v0.2.7)
   - ESP32-TWAI-CAN
   - ArduinoJson (v7.x)

### ライブラリインストール

```bash
# Arduino CLI使用時
arduino-cli lib install "M5StickCPlus2"
arduino-cli lib install "ArduinoJson"
arduino-cli lib install "ESP32-TWAI-CAN"
```

## セットアップ

### 1. WiFi設定

`wifi_config.h`ファイルを作成し、WiFi認証情報を設定：

```cpp
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi設定
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// UDP送信先設定
#define UDP_TARGET_IP "192.168.1.100"
#define POST_UDP_PORT 8888

#endif
```

### 2. コンパイルとアップロード

```bash
# Arduino CLIを使用
arduino-cli compile --fqbn esp32:esp32:m5stick-c-plus2 expo-robomaster-controller.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:m5stick-c-plus2 expo-robomaster-controller.ino
```

## 使用方法

### UDP制御コマンド

ポート`8887`にJSONコマンドを送信してモーターを制御：

```json
{
    "speed": 1.0,        // 回転速度 (rad/s)
    "isRunning": true,   // モーター動作フラグ
    "isTake": false      // 取得フラグ
}
```

#### パラメータ説明

- **speed**: 目標回転速度 (rad/s)。自動的にRPMに変換されます
- **isRunning**: `true`でモーター動作、`false`で停止
- **isTake**: 取得状態のフラグ (用途は実装依存)

### データ出力

モーターとセンサーデータがJSON形式で出力されます：

```json
{
  "motor": {
    "current": 0.5,
    "angle": 180.0,
    "speed": 100,
    "torque": 1024,
    "temp": 25
  },
  "control": {
    "target_rpm": 95.49,
    "current_rpm": 100,
    "output_current": 0.25,
    "error": -4.51
  },
  "accel": {
    "x": 0.123,
    "y": -0.456,
    "z": 9.789
  },
  "gyro": {
    "x": 1.234,
    "y": -2.345,
    "z": 3.456,
    "raw_z": 0.123
  },
  "timestamp": 12345678,
  "counter": 1000
}
```

## 制御パラメータ

### PID制御

```cpp
// NO_LOAD_DEBUG_MODE時
float kp = 0.005;  // 比例ゲイン
float ki = 0.1;    // 積分ゲイン
float kd = 0.0;    // 微分ゲイン

// 通常時
float kp = 0.01;   // 比例ゲイン
float ki = 0.1;    // 積分ゲイン
float kd = 0.0;    // 微分ゲイン
```

### その他の設定

- **最小電流しきい値**: 0.5A (摩擦を超えるための値)
- **ゼロしきい値**: 0.05A (これ以下は停止とみなす)
- **コマンド送信間隔**: 20ms
- **最大出力電流**: ±3.0A

## デバッグモード

コンパイル時のマクロで動作モードを制御：

```cpp
#define SERIAL_DEBUG_MODE    // シリアル出力とJSON データ
#define DEV_DEBUG_MODE       // 開発用デバッグ出力
#define WIFI_DEBUG_MODE      // WiFiスキャンとネットワーク情報
#define NO_LOAD_DEBUG_MODE   // 負荷なしデバッグ (PID調整)
```

## トラブルシューティング

### よくある問題

1. **コンパイルエラー "cpplint command not found"**
   ```bash
   sudo apt install cpplint  # Linux
   pip install cpplint       # Python経由
   ```

2. **CAN通信が確立しない**
   - 配線を確認 (GPIO32, GPIO33)
   - モーターの電源とCAN終端抵抗を確認
   - CAN通信速度 (1Mbps) の設定を確認

3. **WiFi接続失敗**
   - `wifi_config.h`の設定を確認
   - SSIDとパスワードの正確性を確認

4. **ESP32ライブラリ互換性エラー**
   ```bash
   # ESP32 core 3.2.1にダウングレード
   arduino-cli core install esp32:esp32@3.2.1
   ```

### CANプロトコル詳細

- **送信ID**: 0x1FF (モーターコマンド)
- **受信ID**: 0x205 (motor_id=1の場合)
- **通信速度**: 1Mbps
- **データ形式**: 8バイト、電流コマンドは16ビット符号付き整数

### 制御仕様

- **電流範囲**: ±3A (±16384 = 3A)
- **RPM変換**: rad/s × 9.5493 = RPM
- **角度範囲**: 0-360° (8192分解能)
- **温度**: 摂氏 (°C)

## 更新履歴

- **v1.0**: 初期リリース
  - CAN通信とPID制御の実装
  - UDP制御コマンドの対応
  - IMUデータ統合
  - JSON形式でのデータ出力
