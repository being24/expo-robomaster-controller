#include <ArduinoJson.h>
#include <M5StickCPlus2.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <ESP32-TWAI-CAN.hpp>

#include "wifi_config.h"  // WiFi設定を別ファイルから読み込み

#define SERIAL_DEBUG_MODE  // CSV出力やシリアルプロッタ用の出力
// #define DEV_DEBUG_MODE     // 開発用デバッグ出力（受信IDなど）
#define WIFI_DEBUG_MODE  // WiFiスキャンとネットワーク情報の表示

// Default for M5StickC PLUS2
#define CAN_TX 32
#define CAN_RX 33

#define BASE_ID 0x204

// コマンドの送信感覚
#define COMMAND_SEND_INTERVAL 20  // ms
#define LOOP_INTERVAL 10

CanFrame rxFrame;

WiFiUDP udp;
bool wifiConnected = false;

int counter = 0;
bool data_size_error_flag_ = false;

// IMU data variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

constexpr int motor_id = 1;

// PID制御パラメータ（振動を抑えるために調整）
float kp = 0.01;  // 比例ゲイン（減少）
float ki = 0.1;   // 積分ゲイン（減少）
float kd = 0.0;   // 微分ゲイン（無効化）

float target_rpm = 100.0;  // 目標RPM（初期値）
float integral_error = 0.0;
float previous_error = 0.0;
unsigned long last_pid_time = 0;

// 目標RPMの配列（ループで切り替え）
float target_rpm_sequence[] = {50.0, -50.0, 20.0, -20.0};
int target_rpm_index = 0;
const int sequence_length =
    sizeof(target_rpm_sequence) / sizeof(target_rpm_sequence[0]);

// json
StaticJsonDocument<300> doc;

float pid_control(float current_rpm, float target_rpm) {
  unsigned long current_time = millis();
  float dt = (current_time - last_pid_time) / 1000.0f;

  if (dt <= 0.0f) dt = 0.001f;

  // 誤差計算
  float error = target_rpm - current_rpm;

  // 積分項（ワインドアップ抑制付き）
  integral_error += error * dt;
  if (integral_error > 100.0f) integral_error = 100.0f;
  if (integral_error < -100.0f) integral_error = -100.0f;

  // 微分項
  float derivative_error = (error - previous_error) / dt;

  // PID出力計算
  float output = kp * error + ki * integral_error + kd * derivative_error;

  // --- フィードフォワード項（速度比例） ---
  float feedforward = target_rpm * 0.005f;  // 要調整（目安）
  output += feedforward;

  // --- 最小出力しきい値（摩擦を超える） ---
  if (output > 0.0f && output < 0.5f) output = 0.5f;
  if (output < 0.0f && output > -0.5f) output = -0.5f;

  // 出力制限（±3.0A以内）
  if (output > 3.0f) output = 3.0f;
  if (output < -3.0f) output = -3.0f;

  // デッドバンドは無効化して様子を見る
  // if (abs(error) < 5.0f) output = 0.0f;

  previous_error = error;
  last_pid_time = current_time;

  return output;
}

void send_cur(float cur) {
  constexpr float MAX_CUR_A = 3.0f;       // 最大±3A
  constexpr int16_t MAX_CUR_VAL = 16384;  // ±16384 = 3A
  constexpr uint32_t CAN_ID = 0x1FF;      // モーターID 1～4 に対応

  // 電流[A] → 指令値 [-16384, +16384] に変換
  float scaled = cur * (MAX_CUR_VAL / MAX_CUR_A);
  if (scaled > MAX_CUR_VAL) scaled = MAX_CUR_VAL;
  if (scaled < -MAX_CUR_VAL) scaled = -MAX_CUR_VAL;
  int16_t command = static_cast<int16_t>(scaled);

  // CANフレーム作成
  CanFrame frame = {0};
  frame.identifier = CAN_ID;
  frame.extd = 0;
  frame.data_length_code = 8;

  // 8バイト初期化（冗長だが明示的に）
  for (int i = 0; i < 8; i++) frame.data[i] = 0;

  // モーターIDに応じた位置に電流コマンドを格納
  int byteIndex = (motor_id - 1) * 2;
  frame.data[byteIndex] = (command >> 8) & 0xFF;  // 上位バイト
  frame.data[byteIndex + 1] = command & 0xFF;     // 下位バイト

  // フレーム送信
  ESP32Can.writeFrame(frame);

#ifdef DEV_DEBUG_MODE
  // print frame for debugging
  Serial.print("Sending CAN ID: 0x");
  Serial.print(frame.identifier, HEX);
  Serial.print(" Current: ");
  Serial.print(cur);
  Serial.print("A Command: ");
  Serial.print(command);
  Serial.print(" Data: ");
  for (int i = 0; i < frame.data_length_code; i++) {
    Serial.print("0x");
    if (frame.data[i] < 16) Serial.print("0");
    Serial.print(frame.data[i], HEX);
    if (i < frame.data_length_code - 1) Serial.print(", ");
  }
  Serial.println();
#endif
}

void readIMUData() {
  M5.update();
  auto imu_update = M5.Imu.update();
  if (imu_update) {
    auto data = M5.Imu.getImuData();

    // Get acceleration data (m/s^2)
    accelX = data.accel.x;
    accelY = data.accel.y;
    accelZ = data.accel.z;

    // Get gyroscope data (deg/s)
    gyroX = data.gyro.x;
    gyroY = data.gyro.y;
    gyroZ = data.gyro.z;
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);
  M5.Display.setTextFont(&fonts::Orbitron_Light_24);
  M5.Display.setTextSize(0.5);

  // wait 2 seconds for M5StickC to initialize
  delay(2000);

#ifdef SERIAL_DEBUG_MODE
  // Setup serial for debugging.
  Serial.begin(115200);
  while (!Serial);
#endif

  // WiFiスキャンを実行
#ifdef WIFI_DEBUG_MODE
  Serial.println("Scanning for WiFi networks...");

  int numNetworks = WiFi.scanNetworks();

  if (numNetworks == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print(numNetworks);
    Serial.println(" networks found:");

    for (int i = 0; i < numNetworks; i++) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm) ");
      Serial.print(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Open"
                                                            : "Encrypted");
      Serial.println();
    }
  }
  Serial.println();
#endif

  // WiFi接続
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#ifdef SERIAL_DEBUG_MODE
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  Serial.print(" with password ");
  Serial.println(WIFI_PASSWORD);
#endif

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef SERIAL_DEBUG_MODE
    Serial.print(".");
#endif
  }

  wifiConnected = true;
#ifdef SERIAL_DEBUG_MODE
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  // Set pins
  ESP32Can.setPins(CAN_TX, CAN_RX);

  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  // Initialize CAN bus
  if (ESP32Can.begin()) {
#ifdef SERIAL_DEBUG_MODE
    Serial.println("CAN bus started!");
    Serial.println("JSON data output started");  // JSON format notice
#endif
  } else {
#ifdef SERIAL_DEBUG_MODE
    Serial.println("CAN bus failed!");
#endif
    while (1);
  }
}

void loop() {
  int i = 0;

  while (true) {
    // IMUデータを読み取り
    readIMUData();

    // モーター制御 - PID制御で目標RPMに制御
    static unsigned long lastMotorCommand = 0;
    static float current_output = 0.0;
    static float current_rpm = 0.0;
    static unsigned long target_change_time = 0;

    // 5秒ごとに目標RPMを順番に変更（ループ）
    if (millis() - target_change_time >= 5000) {
      target_rpm_index =
          (target_rpm_index + 1) % sequence_length;  // 配列をループ
      target_rpm = target_rpm_sequence[target_rpm_index];
      target_change_time = millis();

      Serial.print(">>> Target RPM changed to: ");
      Serial.print(target_rpm);
      Serial.print(" (");
      Serial.print(target_rpm_index + 1);
      Serial.print("/");
      Serial.print(sequence_length);
      Serial.println(")");

      // PID状態リセット
      integral_error = 0.0;
      previous_error = 0.0;
    }

    if (millis() - lastMotorCommand >= COMMAND_SEND_INTERVAL) {
      current_output = pid_control(current_rpm, target_rpm);
      send_cur(current_output);
      lastMotorCommand = millis();
    }

    // CAN受信処理（sample.txtの効率的な方式を採用）
    if (ESP32Can.readFrame(rxFrame, 1)) {
#ifdef DEV_DEBUG_MODE
      // 開発用デバッグ：受信したすべてのフレームIDを表示
      Serial.print("Received ID: 0x");
      Serial.print(rxFrame.identifier, HEX);
      Serial.print(" Expected: 0x");
      Serial.print(BASE_ID + motor_id, HEX);

      // 受信データも表示
      Serial.print(" RawData: ");
      for (int j = 0; j < rxFrame.data_length_code; j++) {
        Serial.print("0x");
        if (rxFrame.data[j] < 16) Serial.print("0");
        Serial.print(rxFrame.data[j], HEX);
        if (j < rxFrame.data_length_code - 1) Serial.print(" ");
      }
      Serial.println();
#endif

      if (rxFrame.identifier == BASE_ID + motor_id) {  // motor_id=1なので0x205
        counter++;
        data_size_error_flag_ = (rxFrame.data_length_code != 8);

        // CANからの生データ取得
        uint16_t mech_angle = (rxFrame.data[0] << 8) | rxFrame.data[1];
        int16_t speed = (rxFrame.data[2] << 8) | rxFrame.data[3];
        int16_t torque = (rxFrame.data[4] << 8) | rxFrame.data[5];
        uint8_t temp = rxFrame.data[6];

        // --- 正規化 ---
        float angle_deg = mech_angle * 360.0f / 8192.0f;
        float speed_rpm = speed;  // RPM値
        float speed_deg =
            -speed *
            6.0f;  // RPM -> deg/s変換 (RPM * 360deg/60s = RPM * 6) 軸反転
        float current_A = torque / 2048.0f;

        // PID制御のために現在のRPMを更新
        current_rpm = speed_rpm;

        // モーターデータを追加
        doc["motor"]["current"] = current_A;
        doc["motor"]["angle"] = angle_deg;
        doc["motor"]["speed"] = speed_rpm;
        doc["motor"]["torque"] = torque;
        doc["motor"]["temp"] = temp;

        // PID制御情報を追加
        doc["control"]["target_rpm"] = target_rpm;
        doc["control"]["current_rpm"] = speed_rpm;
        doc["control"]["output_current"] = current_output;
        doc["control"]["error"] = target_rpm - speed_rpm;

        // 加速度データをトップレベルに追加
        doc["accel"]["x"] = round(accelX * 1000) / 1000.0;  // 小数点以下3桁
        doc["accel"]["y"] = round(accelY * 1000) / 1000.0;
        doc["accel"]["z"] = round(accelZ * 1000) / 1000.0;

        // ジャイロデータをトップレベルに追加
        doc["gyro"]["x"] = round(gyroX * 1000) / 1000.0;
        doc["gyro"]["y"] = round(gyroY * 1000) / 1000.0;
        doc["gyro"]["raw_z"] =
            round(gyroZ * 1000) / 1000.0;  // IMUのジャイロZ値(deg/s)
        doc["gyro"]["z"] =
            round(speed_deg * 1000) / 1000.0;  // モーターRPM->deg/s変換値

        // タイムスタンプを追加
        doc["timestamp"] = millis();
        doc["counter"] = counter;

        // UDP送信（WiFi接続時のみ）
        if (wifiConnected && WiFi.status() == WL_CONNECTED) {
          String jsonString;
          serializeJson(doc, jsonString);

          udp.beginPacket(UDP_TARGET_IP, UDP_PORT);
          udp.print(jsonString);
          udp.endPacket();
        }

#ifdef SERIAL_DEBUG_MODE
        // JSON形式での出力
        if (!(i % 100)) {  // 出力頻度制限
          // JSONドキュメントを作成

          // JSONを出力
          serializeJson(doc, Serial);
          Serial.println();
        }
#endif
      }
    }

    delay(LOOP_INTERVAL);
    i++;
  }

#ifdef DEV_DEBUG_MODE
  // 開発用デバッグ情報
  Serial.print("counter:");
  Serial.print(counter);
  Serial.print(" error:");
  Serial.println(data_size_error_flag_);
#endif
}