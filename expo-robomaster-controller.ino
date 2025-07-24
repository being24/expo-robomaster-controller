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
#define LOOP_INTERVAL 50

CanFrame rxFrame;

WiFiUDP udp;
bool wifiConnected = false;

int counter = 0;
bool data_size_error_flag_ = false;

// IMU data variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

constexpr int motor_id = 1;

// json
StaticJsonDocument<300> doc;

void send_cur(float cur) {
  constexpr float MAX_CUR = 10;
  constexpr int MAX_CUR_VAL = 10000;

  float val = cur * (MAX_CUR_VAL / MAX_CUR);
  if (val < -MAX_CUR_VAL)
    val = -MAX_CUR_VAL;
  else if (val > MAX_CUR_VAL)
    val = MAX_CUR_VAL;
  int16_t transmit_val = val;

  CanFrame obdFrame = {0};
  obdFrame.identifier = BASE_ID + motor_id;
  obdFrame.extd = 0;
  obdFrame.data_length_code = 8;

  // Initialize all data to 0
  for (int i = 0; i < 8; i++) {
    obdFrame.data[i] = 0;
  }

  // Set motor current data
  obdFrame.data[(motor_id - 1) * 2] = (transmit_val >> 8) & 0xFF;
  obdFrame.data[(motor_id - 1) * 2 + 1] = transmit_val & 0xFF;

  ESP32Can.writeFrame(obdFrame);
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

    // CAN受信処理（sample.txtの効率的な方式を採用）
    if (ESP32Can.readFrame(rxFrame, 1)) {
#ifdef DEV_DEBUG_MODE
      // 開発用デバッグ：受信したすべてのフレームIDを表示
      Serial.print("Received ID: 0x");
      Serial.print(rxFrame.identifier, HEX);
      Serial.print(" Expected: 0x");
      Serial.println(BASE_ID + motor_id, HEX);
#endif

      if (rxFrame.identifier == BASE_ID + motor_id) {  // motor_id=1なので0x205
        counter++;
        data_size_error_flag_ = (rxFrame.data_length_code != 8);

        // Parse feedback data (sample.txtと同様の方式)
        uint16_t mech_angle = (rxFrame.data[0] << 8) | rxFrame.data[1];
        int16_t speed = (rxFrame.data[2] << 8) | rxFrame.data[3];
        int16_t torque = (rxFrame.data[4] << 8) | rxFrame.data[5];
        uint8_t temp = rxFrame.data[6];

        // モーターデータを追加
        doc["motor"]["current"] = 0;  // 電流値は0（出力なし）
        doc["motor"]["angle"] = mech_angle;
        doc["motor"]["speed"] = speed;
        doc["motor"]["torque"] = torque;
        doc["motor"]["temp"] = temp;

        // 加速度データをトップレベルに追加
        doc["accel"]["x"] = round(accelX * 1000) / 1000.0;  // 小数点以下3桁
        doc["accel"]["y"] = round(accelY * 1000) / 1000.0;
        doc["accel"]["z"] = round(accelZ * 1000) / 1000.0;

        // ジャイロデータをトップレベルに追加
        doc["gyro"]["x"] = round(gyroX * 1000) / 1000.0;
        doc["gyro"]["y"] = round(gyroY * 1000) / 1000.0;
        doc["gyro"]["z"] = round(gyroZ * 1000) / 1000.0;

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