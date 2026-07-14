#include <ArduinoJson.h>
#include <ETH.h>
#include <M5Unified.h>
#include <Network.h>
#include <NetworkUdp.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// airi
#define SERIAL_DEBUG_MODE
#define ENABLE_DISPLAY
// #define UDP_LOG_MODE
// #define DEV_DEBUG_MODE      // 開発用デバッグ出力（受信IDなど）
// #define NO_LOAD_DEBUG_MODE  // 負荷なしデバッグ

#include <ESP32-TWAI-CAN.hpp>

#include "wifi_config.h"  // 既存の UDP_TARGET_IP / RECEIVE_UDP_PORT / POST_UDP_PORT を流用

// =========================
// AtomS3 / CAN
// =========================
#define CAN_TX 2
#define CAN_RX 1

#define BASE_ID 0x204

// コマンドの送信間隔
#define COMMAND_SEND_INTERVAL 20  // ms
#define LOOP_INTERVAL 10

// W5500 LAN (SPI) - AtomS3
// G5  -> SCK
// G6  -> CS
// G7  -> MISO
// G8  -> MOSI

#define ETH_PHY_TYPE ETH_PHY_W5500
#define ETH_PHY_ADDR 1
#define ETH_PHY_CS 6
#define ETH_PHY_IRQ -1
#define ETH_PHY_RST -1

#define ETH_SPI_SCK 5
#define ETH_SPI_MISO 7
#define ETH_SPI_MOSI 8

static volatile bool lanLinkUp = false;
static volatile bool lanHasIP = false;
static volatile bool networkReady = false;
static volatile bool udpCommandServerStarted = false;
static volatile bool udpSendSocketStarted = false;
static volatile bool ethNeedsSocketSync = true;

#define DISPLAY_REFRESH_INTERVAL 200  // 200ms = 5Hz
#define DISPLAY_BRIGHTNESS 48         // 先保守一点，0~255

CanFrame rxFrame;

NetworkUDP udp;
NetworkUDP commandUdp;  // コマンド受信用（ETH 上で使用）
IPAddress currentBoundIP(0, 0, 0, 0);

int counter = 0;
bool data_size_error_flag_ = false;

// IMU data variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

constexpr int motor_id = 1;

// 測定済みの電流-RPMマッピングデータ
static const float mapping_current[] = {
    0.0,  0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7,  0.8,  0.9,  1.0,  1.2,
    1.4,  1.6,  1.8,  2.0,  2.5,  3.0,  -0.1, -0.2, -0.3, -0.4, -0.5, -0.6,
    -0.7, -0.8, -0.9, -1.0, -1.2, -1.4, -1.6, -1.8, -2.0, -2.5, -3.0};

static const float mapping_rpm[] = {
    0.00,   0.00,   0.00,   0.00,   0.02,   0.08,   1.49,   4.68,  6.72,
    8.91,   11.65,  17.05,  22.74,  29.08,  34.89,  41.38,  56.11, 72.33,
    0.00,   -0.00,  -0.01,  -0.02,  -0.09,  -1.30,  -3.35,  -4.91, -7.42,
    -10.29, -14.45, -20.66, -25.99, -31.43, -37.67, -51.54, -66.49};

static const int mapping_size = sizeof(mapping_current) / sizeof(float);

// RPMから電流値を線形補間で計算する関数
float rpmToCurrent(float target_rpm) {
  float min_rpm = -66.49;
  float max_rpm = 72.33;

  if (target_rpm > max_rpm) target_rpm = max_rpm;
  if (target_rpm < min_rpm) target_rpm = min_rpm;

  if (abs(target_rpm) < 0.1) {
    return 0.0;
  }

  if (target_rpm > 0) {
    for (int i = 0; i < 18; i++) {
      if (mapping_rpm[i] >= target_rpm) {
        if (i == 0 || mapping_rpm[i - 1] == mapping_rpm[i]) {
          return mapping_current[i];
        }

        float ratio = (target_rpm - mapping_rpm[i - 1]) /
                      (mapping_rpm[i] - mapping_rpm[i - 1]);
        return mapping_current[i - 1] +
               ratio * (mapping_current[i] - mapping_current[i - 1]);
      }
    }
    return mapping_current[17];
  } else {
    for (int i = 18; i < mapping_size; i++) {
      if (mapping_rpm[i] <= target_rpm) {
        if (i == 18 || mapping_rpm[i - 1] == mapping_rpm[i]) {
          return mapping_current[i];
        }

        float ratio = (target_rpm - mapping_rpm[i - 1]) /
                      (mapping_rpm[i] - mapping_rpm[i - 1]);
        return mapping_current[i - 1] +
               ratio * (mapping_current[i] - mapping_current[i - 1]);
      }
    }
    return mapping_current[mapping_size - 1];
  }
}

// 簡易的なRPM制御関数（PIDの代替）
float simpleRpmControl(float current_rpm, float target_rpm) {
  (void)current_rpm;
  float base_current = rpmToCurrent(target_rpm);
  return base_current;
}

// PID制御パラメータ
#ifdef NO_LOAD_DEBUG_MODE
float kp = 0.005;
float ki = 0.1;
float kd = 0.0;
#else
float kp = 0.01;
float ki = 0.1;
float kd = 0.0;
#endif

float min_current = 0.5;
float zero_threshold = 0.05;

bool is_running = false;
bool is_take = false;
float target_rpm = 0.0;
float current_rpm = 0.0;
float integral_error = 0.0;
float previous_error = 0.0;
unsigned long last_pid_time = 0;

// 目標RPMの配列（ループで切り替え）
// float target_rpm_sequence[] = {50.0, -50.0, 20.0, -20.0};
// int target_rpm_index = 0;
// const int sequence_length =
//     sizeof(target_rpm_sequence) / sizeof(target_rpm_sequence[0]);

// json
StaticJsonDocument<300> doc;
StaticJsonDocument<200> commandDoc;

// UDP受信タスク用変数
SemaphoreHandle_t udpDataMutex;
SemaphoreHandle_t udpSocketMutex;
bool newUdpDataAvailable = false;
float latest_angular_velocity = 0.0;
bool latest_is_running = false;
bool latest_is_take = false;

// 角度からRPM計算用
static float prev_angle = 0.0;
static unsigned long prev_time = 0;
static bool angle_calc_initialized = false;

// RPM移動平均フィルター（5点平均）
#define RPM_FILTER_SIZE 5
static float rpm_buffer[RPM_FILTER_SIZE] = {0};
static int rpm_buffer_idx = 0;
static bool rpm_buffer_full = false;

// =========================
// Ethernet event handler
// =========================
void onNetworkEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
#ifdef SERIAL_DEBUG_MODE
      Serial.println("ETH Started");
#endif
      ETH.setHostname("atoms3-eth");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      lanLinkUp = true;
#ifdef SERIAL_DEBUG_MODE
      Serial.println("ETH Link Up");
#endif
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      lanLinkUp = true;
      lanHasIP = true;
      networkReady = true;
      ethNeedsSocketSync = true;
#ifdef SERIAL_DEBUG_MODE
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
#endif
      break;

    case ARDUINO_EVENT_ETH_LOST_IP:
      lanHasIP = false;
      networkReady = false;
      ethNeedsSocketSync = true;
#ifdef SERIAL_DEBUG_MODE
      Serial.println("ETH Lost IP");
#endif
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      lanLinkUp = false;
      lanHasIP = false;
      networkReady = false;
      ethNeedsSocketSync = true;
#ifdef SERIAL_DEBUG_MODE
      Serial.println("ETH Disconnected");
#endif
      break;

    case ARDUINO_EVENT_ETH_STOP:
      lanLinkUp = false;
      lanHasIP = false;
      networkReady = false;
      ethNeedsSocketSync = true;
#ifdef SERIAL_DEBUG_MODE
      Serial.println("ETH Stopped");
#endif
      break;

    default:
      break;
  }
}

bool isValidIP(const IPAddress& ip) {
  return (ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0);
}

void stopUdpSocketsUnlocked() {
  if (udpCommandServerStarted) {
    commandUdp.stop();
    udpCommandServerStarted = false;
#ifdef SERIAL_DEBUG_MODE
    Serial.println("UDP command server stopped");
#endif
  }

  if (udpSendSocketStarted) {
    udp.stop();
    udpSendSocketStarted = false;
#ifdef SERIAL_DEBUG_MODE
    Serial.println("UDP sender socket stopped");
#endif
  }

  currentBoundIP = IPAddress(0, 0, 0, 0);
}

void stopUdpSockets() {
  if (xSemaphoreTake(udpSocketMutex, portMAX_DELAY) == pdTRUE) {
    stopUdpSocketsUnlocked();
    xSemaphoreGive(udpSocketMutex);
  }
}

void startUdpSockets(const IPAddress& ip) {
  if (xSemaphoreTake(udpSocketMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  stopUdpSocketsUnlocked();

  bool rxOk = commandUdp.begin(RECEIVE_UDP_PORT);
  bool txOk = udp.begin(SEND_UDP_PORT);

  udpCommandServerStarted = rxOk;
  udpSendSocketStarted = txOk;

  if (rxOk && txOk) {
    currentBoundIP = ip;
    ethNeedsSocketSync = false;
#ifdef SERIAL_DEBUG_MODE
    Serial.print("UDP sockets bound on IP: ");
    Serial.println(ip);
    Serial.print("RX port: ");
    Serial.println(RECEIVE_UDP_PORT);
    Serial.print("TX local port: ");
    Serial.println(SEND_UDP_PORT);
#endif
  } else {
    ethNeedsSocketSync = true;
#ifdef SERIAL_DEBUG_MODE
    Serial.print("UDP socket bind failed. RX=");
    Serial.print(rxOk ? "OK" : "NG");
    Serial.print(" TX=");
    Serial.println(txOk ? "OK" : "NG");
#endif
  }

  xSemaphoreGive(udpSocketMutex);
}

void syncUdpSockets() {
  static unsigned long lastRetryTime = 0;

  if (!lanHasIP && isValidIP(ETH.localIP())) {
    lanHasIP = true;
    networkReady = true;
    ethNeedsSocketSync = true;
  }

  IPAddress ip = ETH.localIP();
  bool ipReady = lanHasIP && isValidIP(ip);

  if (!ipReady) {
    stopUdpSockets();
    return;
  }

  bool ipChanged = (ip != currentBoundIP);

  if (!ethNeedsSocketSync && !ipChanged && udpCommandServerStarted &&
      udpSendSocketStarted) {
    return;
  }

  if (millis() - lastRetryTime < 1000) {
    return;
  }
  lastRetryTime = millis();

  startUdpSockets(ip);
}

// UDP受信タスク
void udpReceiveTask(void* parameter) {
  (void)parameter;

  while (true) {
    if (!udpCommandServerStarted) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    char incomingPacket[256];
    int len = 0;

    if (xSemaphoreTake(udpSocketMutex, portMAX_DELAY) != pdTRUE) {
      vTaskDelay(1);
      continue;
    }

    int packetSize = 0;
    if (udpCommandServerStarted) {
      packetSize = commandUdp.parsePacket();
      if (packetSize) {
        len = commandUdp.read(incomingPacket, sizeof(incomingPacket) - 1);
      }
    }

    xSemaphoreGive(udpSocketMutex);

    if (packetSize) {
#ifdef UDP_LOG_MODE
      Serial.println(">>>> UDP Packet Received! <<<<");
#endif

      if (len > 0) {
        incomingPacket[len] = 0;
#ifdef UDP_LOG_MODE
        Serial.printf("Received Data: %s\n", incomingPacket);
#endif

        DeserializationError receive_data =
            deserializeJson(commandDoc, incomingPacket);
        if (!receive_data) {
          if (xSemaphoreTake(udpDataMutex, portMAX_DELAY)) {
            if (commandDoc.containsKey("angular_velocity")) {
              latest_angular_velocity = commandDoc["angular_velocity"];
            }
            if (commandDoc.containsKey("isTake")) {
              latest_is_take = commandDoc["isTake"];
            }
            if (commandDoc.containsKey("isRunning")) {
              latest_is_running = commandDoc["isRunning"];
            }
            newUdpDataAvailable = true;
            xSemaphoreGive(udpDataMutex);
          }
        } else {
#ifdef SERIAL_DEBUG_MODE
          Serial.print("JSON parse receive_data: ");
          Serial.println(receive_data.c_str());
#endif
        }
      }
    }
    vTaskDelay(1);
  }
}

void processUdpCommand() {
  if (xSemaphoreTake(udpDataMutex, 0) == pdTRUE) {
    if (newUdpDataAvailable) {
      target_rpm = latest_angular_velocity * 9.5493;  // rad/s -> RPM
      is_take = latest_is_take;
      is_running = latest_is_running;

      newUdpDataAvailable = false;

#ifdef SERIAL_DEBUG_MODE
      static unsigned long lastUdpDebugTime = 0;
      if (millis() - lastUdpDebugTime >= 1000) {
        Serial.print("UDP Command Applied - angular_velocity=");
        Serial.print(latest_angular_velocity);
        Serial.print(" rad/s, target_rpm=");
        Serial.print(target_rpm);
        Serial.print(", take=");
        Serial.print(is_take ? "ON" : "OFF");
        Serial.print(", running=");
        Serial.println(is_running ? "ON" : "OFF");
        lastUdpDebugTime = millis();
      }
#endif
    }
    xSemaphoreGive(udpDataMutex);
  }
}

// 角度から瞬間RPMを計算
float calculateInstantRpm(float current_angle, unsigned long current_time) {
  if (!angle_calc_initialized) {
    prev_angle = current_angle;
    prev_time = current_time;
    angle_calc_initialized = true;
    return 0.0;
  }

  unsigned long time_diff = current_time - prev_time;
  if (time_diff == 0) return 0.0;

  float angle_diff = current_angle - prev_angle;
  if (angle_diff > 180.0)
    angle_diff -= 360.0;
  else if (angle_diff < -180.0)
    angle_diff += 360.0;

  float instant_rpm = (angle_diff / (float)time_diff) * 1000.0 * 60.0 / 360.0;

  prev_angle = current_angle;
  prev_time = current_time;

  return instant_rpm;
}

// RPM移動平均フィルター
float applyRpmFilter(float new_rpm) {
  rpm_buffer[rpm_buffer_idx] = new_rpm;
  rpm_buffer_idx = (rpm_buffer_idx + 1) % RPM_FILTER_SIZE;

  if (!rpm_buffer_full && rpm_buffer_idx == 0) {
    rpm_buffer_full = true;
  }

  float sum = 0.0;
  int count = rpm_buffer_full ? RPM_FILTER_SIZE : rpm_buffer_idx;
  for (int i = 0; i < count; i++) {
    sum += rpm_buffer[i];
  }

  return (count > 0) ? (sum / count) : 0.0f;
}

float pid_control(float current_rpm, float target_rpm) {
  if (!is_running) {
    integral_error = 0.0;
    return 0.0f;
  }

  unsigned long current_time = millis();
  float dt = (current_time - last_pid_time) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  float error = target_rpm - current_rpm;

  static float previous_target_rpm = target_rpm;
  if (abs(target_rpm - previous_target_rpm) > 0.5f) {
    integral_error = 0.0;
#ifdef SERIAL_DEBUG_MODE
    Serial.println("Target changed - Integral reset");
#endif
  }
  previous_target_rpm = target_rpm;

  static float previous_error_sign = 0.0;
  float current_error_sign = (error > 0) ? 1.0 : (error < 0) ? -1.0 : 0.0;
  bool is_braking = (previous_error_sign != 0 && current_error_sign != 0 &&
                     previous_error_sign != current_error_sign);

  if (is_braking) {
    integral_error = 0.0;
#ifdef SERIAL_DEBUG_MODE
    Serial.println("Braking detected - Integral reset");
#endif
  }
  previous_error_sign = current_error_sign;

  if (!is_braking) {
    integral_error += error * dt;
  }

  float max_integral = 100.0f;
  if (integral_error > max_integral) integral_error = max_integral;
  if (integral_error < -max_integral) integral_error = -max_integral;

  float derivative_error = (error - previous_error) / dt;
  float output = kp * error + ki * integral_error + kd * derivative_error;

  float feedforward = target_rpm * 0.005f;
  output += feedforward;

  if (output > -zero_threshold && output < zero_threshold)
    output = 0.0f;
  else if (output > 0.0f && output < min_current)
    output = min_current;
  else if (output < 0.0f && output > -min_current)
    output = -min_current;

  if (output > 3.0f) output = 3.0f;
  if (output < -3.0f) output = -3.0f;

  previous_error = error;
  last_pid_time = current_time;

  return output;
}

void send_cur(float cur) {
  constexpr float MAX_CUR_A = 3.0f;
  constexpr int16_t MAX_CUR_VAL = 16384;
  constexpr uint32_t CAN_ID = 0x1FF;

  float scaled = cur * (MAX_CUR_VAL / MAX_CUR_A);
  if (scaled > MAX_CUR_VAL) scaled = MAX_CUR_VAL;
  if (scaled < -MAX_CUR_VAL) scaled = -MAX_CUR_VAL;
  int16_t command = static_cast<int16_t>(scaled);

  CanFrame frame = {0};
  frame.identifier = CAN_ID;
  frame.extd = 0;
  frame.data_length_code = 8;

  for (int i = 0; i < 8; i++) frame.data[i] = 0;

  int byteIndex = (motor_id - 1) * 2;
  frame.data[byteIndex] = (command >> 8) & 0xFF;
  frame.data[byteIndex + 1] = command & 0xFF;

  ESP32Can.writeFrame(frame);

#ifdef DEV_DEBUG_MODE
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

    accelX = data.accel.x;
    accelY = data.accel.y;
    accelZ = data.accel.z;

    gyroX = data.gyro.x;
    gyroY = data.gyro.y;
    gyroZ = data.gyro.z;
  }
}

void updateDisplay() {
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate < DISPLAY_REFRESH_INTERVAL) return;
  lastDisplayUpdate = millis();

  M5.Display.startWrite();
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);

  M5.Display.printf("IP: %s\n\n", ETH.localIP().toString().c_str());
  M5.Display.printf("LAN: %s / %s\n", lanLinkUp ? "LINK" : "DOWN",
                    lanHasIP ? "IP OK" : "NO IP");
  M5.Display.printf("Target:  %.2f RPM\n", target_rpm);
  M5.Display.printf("Current: %.2f RPM\n", current_rpm);
  M5.Display.printf("Running: %s\n", is_running ? "ON" : "OFF");
  M5.Display.printf("Is Take: %s\n", is_take ? "ON" : "OFF");

  M5.Display.endWrite();
}

void setup() {
#ifdef SERIAL_DEBUG_MODE
  Serial.begin(115200);
  unsigned long serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start < 2000)) {
    delay(10);
  }
#endif

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);
  M5.Display.setTextFont(&fonts::Orbitron_Light_24);
  M5.Display.setTextSize(0.5);
  M5.Display.setBrightness(DISPLAY_BRIGHTNESS);  // 太亮了发热很严重

  delay(500);

  // ミューテックス作成
  udpDataMutex = xSemaphoreCreateMutex();
  udpSocketMutex = xSemaphoreCreateMutex();

  if (udpDataMutex == NULL || udpSocketMutex == NULL) {
#ifdef SERIAL_DEBUG_MODE
    Serial.println("Failed to create UDP mutex");
#endif
    while (1) {
      delay(1000);
    }
  }

  // UDP受信タスクを作成
  xTaskCreatePinnedToCore(udpReceiveTask, "UDPReceiveTask", 4096, NULL, 2, NULL,
                          0);

  // Ethernet(W5500) 初期化
  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  SPI.setFrequency(10000000);  // 少し保守的に

  Network.onEvent(onNetworkEvent);

#ifdef SERIAL_DEBUG_MODE
  Serial.println("Starting Ethernet (W5500 over SPI)...");
#endif

  if (!ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ,
                 ETH_PHY_RST, SPI)) {
#ifdef SERIAL_DEBUG_MODE
    Serial.println("ETH.begin() failed!");
#endif
    while (1) {
      delay(1000);
    }
  }

  // 固定IP設定（DHCPを使わない）
  if (USE_STATIC_IP) {
    bool cfgOk = ETH.config(LOCAL_IP, LOCAL_GATEWAY, LOCAL_SUBNET, LOCAL_DNS1,
                            LOCAL_DNS2);
#ifdef SERIAL_DEBUG_MODE
    Serial.print("ETH.config(static IP) = ");
    Serial.println(cfgOk ? "OK" : "NG");
#endif
  }

  // 起動時に少し待つ（固定IP反映 / DHCP取得待ち）
  unsigned long eth_wait_start = millis();
  while ((millis() - eth_wait_start < 5000)) {
    syncUdpSockets();

    if (USE_STATIC_IP) {
      if (ETH.localIP() == LOCAL_IP) {
        lanHasIP = true;
        networkReady = true;
        ethNeedsSocketSync = true;
        break;
      }
    } else {
      if (lanHasIP) {
        break;
      }
    }

    delay(50);
  }

#ifdef SERIAL_DEBUG_MODE
  Serial.print("Ethernet local IP: ");
  Serial.println(ETH.localIP());
#endif

  // CAN 設定
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  if (ESP32Can.begin()) {
#ifdef SERIAL_DEBUG_MODE
    Serial.println("CAN bus started!");
    Serial.println("JSON data output started");
#endif
  } else {
#ifdef SERIAL_DEBUG_MODE
    Serial.println("CAN bus failed!");
#endif
    while (1) {
      delay(1000);
    }
  }
}

void loop() {
  int i = 0;

  while (true) {
    syncUdpSockets();

    readIMUData();
    processUdpCommand();

#ifdef ENABLE_DISPLAY
    updateDisplay();
#endif

    static unsigned long lastMotorCommand = 0;
    static float current_output = 0.0;
    static unsigned long target_change_time = 0;

    // 5秒ごとに目標RPMを順番に変更（ループ）
    //     if (millis() - target_change_time >= 5000) {
    //       target_rpm_index =
    //           (target_rpm_index + 1) % sequence_length;  // 配列をループ
    //       target_rpm = target_rpm_sequence[target_rpm_index];
    //       target_change_time = millis();

    // #ifdef SERIAL_DEBUG_MODE
    //       Serial.print(">>> Target RPM changed to: ");
    //       Serial.print(target_rpm);
    //       Serial.print(" (");
    //       Serial.print(target_rpm_index + 1);
    //       Serial.print("/");
    //       Serial.print(sequence_length);
    //       Serial.println(")");
    // #endif

    //       // PID状態リセット
    //       integral_error = 0.0;
    //       previous_error = 0.0;
    //     }

    if (millis() - lastMotorCommand >= COMMAND_SEND_INTERVAL) {
      current_output = simpleRpmControl(current_rpm, target_rpm);
      send_cur(current_output);
      lastMotorCommand = millis();
    }

    if (ESP32Can.readFrame(rxFrame, 1)) {
#ifdef DEV_DEBUG_MODE
      Serial.print("Received ID: 0x");
      Serial.print(rxFrame.identifier, HEX);
      Serial.print(" Expected: 0x");
      Serial.print(BASE_ID + motor_id, HEX);

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
        float speed_rpm_raw = speed;  // モーターから直接取得したRPM

        // 角度から瞬間RPMを計算
        float instant_rpm = calculateInstantRpm(angle_deg, millis());

        // 移動平均フィルターを適用
        float speed_rpm = applyRpmFilter(instant_rpm);

        float speed_deg = -speed_rpm * 6.0f;  // 角度計算RPM -> deg/s変換 (RPM *
                                              // 360deg/60s = RPM * 6) 軸反転
        float current_A = torque / 2048.0f;

        // PID制御のために角度計算RPMを使用
        current_rpm = speed_rpm;

        // モーターデータを追加
        doc["motor"]["current"] = current_A;
        doc["motor"]["angle"] = angle_deg;
        doc["motor"]["speed"] = speed_rpm;
        doc["motor"]["speed_raw"] = speed_rpm_raw;
        doc["motor"]["speed_instant"] = instant_rpm;
        doc["motor"]["torque"] = torque;
        doc["motor"]["temp"] = temp;

        // PID制御情報を追加
        doc["control"]["target_rpm"] = target_rpm;
        doc["control"]["current_rpm"] = speed_rpm;
        doc["control"]["output_current"] = current_output;
        doc["control"]["error"] = target_rpm - speed_rpm;

        // 加速度データをトップレベルに追加
        doc["accel"]["x"] = round(accelX * 1000) / 1000.0;
        doc["accel"]["y"] = round(accelY * 1000) / 1000.0;
        doc["accel"]["z"] = round(accelZ * 1000) / 1000.0;

        // ジャイロデータをトップレベルに追加
        doc["gyro"]["x"] = round(gyroX * 1000) / 1000.0;
        doc["gyro"]["y"] = round(gyroY * 1000) / 1000.0;
        doc["gyro"]["raw_z"] = round(gyroZ * 1000) / 1000.0;
        doc["gyro"]["z"] = round(speed_deg * 1000) / 1000.0;

        // タイムスタンプを追加
        doc["timestamp"] = millis();
        doc["counter"] = counter;

        // UDP送信（LAN接続時のみ）
        if (lanHasIP && udpSendSocketStarted) {
          char jsonBuffer[512];
          size_t jsonLen = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

          if (jsonLen == 0 || jsonLen >= sizeof(jsonBuffer)) {
#ifdef SERIAL_DEBUG_MODE
            Serial.println("serializeJson failed or buffer too small");
#endif
          } else {
            if (!udp.beginPacket(UDP_TARGET_IP, POST_UDP_PORT)) {
#ifdef SERIAL_DEBUG_MODE
              Serial.println("udp.beginPacket failed");
#endif
              udpSendSocketStarted = false;
              ethNeedsSocketSync = true;
            } else {
              size_t written = udp.write((const uint8_t*)jsonBuffer, jsonLen);
              if (written != jsonLen) {
#ifdef SERIAL_DEBUG_MODE
                Serial.println("udp.write size mismatch");
#endif
              }

              if (!udp.endPacket()) {
#ifdef SERIAL_DEBUG_MODE
                Serial.println("udp.endPacket failed");
#endif
                udpSendSocketStarted = false;
                ethNeedsSocketSync = true;
              }
            }
          }
        }

#ifdef SERIAL_DEBUG_MODE
        if (!(i % 100)) {
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
