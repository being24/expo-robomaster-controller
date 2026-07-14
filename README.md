# Expo RoboMaster Controller

M5Stack AtomS3でRoboMasterモーターを制御し、Atomic PoE（W5500）経由でUDP通信するプロジェクトです。

ルートの`expo-robomaster-controller.ino`が通常使用するモーターノードです。`ino/`には動作確認用スケッチと退避したスケッチがあります。

## 構成

### モーターノード

- M5Stack AtomS3
- M5Stack Atomic PoE Base V1.2.1
- M5Stack Unit CAN（U085）
- RoboMasterモーター
- 固定IP: `192.168.1.30`
- UDPコマンド受信ポート: `9000`
- UDPテレメトリー送信先: `UDP_TARGET_IP:9001`
- UDP送信元ポート: `9002`

### 距離センサーノード

CANとI2CはAtomS3の同じPort Aを使用するため、MINITOF-90は予備のAtomS3＋Atomic PoEへ接続します。

- M5Stack AtomS3
- M5Stack Atomic PoE Base V1.2.1
- M5Stack Unit Mini ToF-90°（VL53L0X）
- 使用スケッチ: `ino/distance-sensor.ino`
- 固定IP: `192.168.1.31`
- UDPテレメトリー送信先: `UDP_TARGET_IP:9011`
- UDP送信元ポート: `9012`
- UDP受信: なし

両ノードの送信先IPは`wifi_config.h`の`UDP_TARGET_IP`で共通です。ポート番号でデータを分離します。

## ピン割り当て

### Atomic PoE（両ノード共通）

| 信号 | AtomS3 GPIO |
|---|---:|
| SPI SCK | 5 |
| W5500 CS | 6 |
| SPI MISO | 7 |
| SPI MOSI | 8 |

### モーターノードのUnit CAN

| 信号 | AtomS3 GPIO |
|---|---:|
| CAN TX | 2 |
| CAN RX | 1 |

CAN通信速度は1Mbpsです。モーターID 1のフィードバックIDは`0x205`、電流指令IDは`0x1FF`です。

### 距離センサーノードのMINITOF-90

| 信号 | AtomS3 GPIO |
|---|---:|
| I2C SDA | 2 |
| I2C SCL | 1 |

Unit CANとMINITOF-90をパッシブGrove HUBで同じAtomS3へ同時接続しないでください。GPIO1/2上でTWAI（CAN）とI2Cが競合します。

## セットアップ

`arduino-cli`と`jq`が必要です。ボード、ESP32コア、ライブラリは初回のみインストールします。

```bash
./bin/init.sh
```

インストールされる主なバージョンは次のとおりです。

- ESP32 Arduino Core `3.2.1`
- M5Unified `0.2.18`
- ArduinoJson `7.4.2`
- ESP32-TWAI-CAN `1.0.1`
- VL53L0X `1.3.1`

## ネットワーク設定

`wifi_config.h`という名前ですが、現在はWi-FiではなくAtomic PoEの有線LAN設定に使用しています。

```cpp
static IPAddress UDP_TARGET_IP(192, 168, 1, 7);

const bool USE_STATIC_IP = true;
static IPAddress LOCAL_IP(192, 168, 1, 30);
static IPAddress LOCAL_GATEWAY(192, 168, 1, 1);
static IPAddress LOCAL_SUBNET(255, 255, 255, 0);
static IPAddress LOCAL_DNS1(192, 168, 1, 1);
static IPAddress LOCAL_DNS2(8, 8, 8, 8);

const uint16_t POST_UDP_PORT = 9001;
const uint16_t RECEIVE_UDP_PORT = 9000;
const uint16_t SEND_UDP_PORT = 9002;
```

距離センサーノードは送信先IP、ゲートウェイ、サブネット、DNSをこのファイルから流用し、自身のIPとUDPポートをスケッチ内で`192.168.1.31`、`9011/9012`へ上書きしています。

## ビルドと書き込み

`.vscode/arduino.json`の`board`と`configuration`を使用します。書き込み時は同ファイルの`port`も使用します。

```bash
# ルートのスケッチをテストビルド
./bin/verify.sh

# ルートのスケッチをビルドして書き込み
./bin/write.sh
```

スクリプトが対象にするのは、必ずルートの`expo-robomaster-controller.ino`です。`ino/`のテストを使う場合は、現在のルートスケッチを保護したうえで、対象の内容をルートスケッチへコピーしてから実行します。

## UDPコマンド

モーターノードの`192.168.1.30:9000`へJSONを送ります。

```json
{
  "angular_velocity": 3.14,
  "isRunning": true,
  "isTake": false
}
```

- `angular_velocity`: 目標角速度（rad/s）。内部で`rad/s × 9.5493`によりRPMへ変換
- `isRunning`: 現在は状態表示用。停止指令では`angular_velocity`も`0`にする
- `isTake`: 取得状態フラグ

現在の出力電流は、目標RPMに対応する実測マッピングから求めています。PID関数はコード内に残っていますが、通常ループでは使用していません。

## モーターテレメトリー

モーターからCANフィードバックを受けたとき、`UDP_TARGET_IP:9001`へJSONを送信します。

```json
{
  "motor": {
    "current": 0.5,
    "angle": 180.0,
    "speed": 28.4,
    "speed_raw": 29,
    "speed_instant": 30.1,
    "torque": 1024,
    "temp": 25
  },
  "control": {
    "target_rpm": 30.0,
    "current_rpm": 28.4,
    "output_current": 1.4,
    "error": 1.6
  },
  "accel": {"x": 0.0, "y": 0.0, "z": 1.0},
  "gyro": {"x": 0.0, "y": 0.0, "raw_z": 0.0, "z": -170.4},
  "timestamp": 123456,
  "counter": 1000
}
```

- `motor.speed`: 角度差から求めたRPMの5点移動平均
- `motor.speed_instant`: 角度差から求めた瞬時RPM
- `motor.speed_raw`: モーターのCANフィードバックに含まれる速度値
- `control.current_rpm`: 制御側で使用する5点移動平均RPM

AtomS3の画面にはIP、LAN状態、目標RPM、瞬時RPM、状態フラグを表示します。

## MINITOF-90テレメトリー

距離センサーノードは100ms間隔で`UDP_TARGET_IP:9011`へ送信します。同じJSONをシリアルコンソールにも出力します。

```json
{
  "sensor": "MINITOF-90",
  "distance_mm": 123,
  "valid": true,
  "timeout": false,
  "sample_count": 100,
  "timeout_count": 0,
  "timestamp": 123456,
  "counter": 50
}
```

## 保存スケッチ

| ファイル | 用途 |
|---|---|
| `expo-robomaster-controller.ino` | 現在ビルドされるモーター制御スケッチ |
| `ino/motor.ino` | モーター制御スケッチの退避コピー |
| `ino/motor-ramp-test.ino` | モーターを徐々に加速・減速する動作確認 |
| `ino/distance-sensor.ino` | MINITOF-90計測とAtomic PoE UDP送信 |

## トラブルシューティング

### CANフィードバックがない

- Unit CANがGPIO2（TX）/ GPIO1（RX）へ接続されているか確認
- CAN H/CAN L、モーター電源、終端抵抗を確認
- モーターIDが1であることを確認
- 通信速度が1Mbpsであることを確認

### `TOF NOT FOUND`

- MINITOF-90を距離センサーノードのPort Aへ単独で接続
- Groveの5V/GND/SDA/SCLを確認
- I2Cアドレス`0x29`の応答がない場合に表示されます

### `TOF INIT FAILED`

I2Cアドレス`0x29`には応答していますが、VL53L0Xの初期化に失敗しています。電源を入れ直し、別のGroveケーブルでも確認してください。

### UDPが届かない

- 画面で固定IPと`LAN: LINK / IP OK`を確認
- モーターノードと距離センサーノードで固定IPが重複していないか確認
- 受信側でUDPポート`9001`と`9011`の両方を待ち受ける
- `UDP_TARGET_IP`、サブネット、ファイアウォールを確認
