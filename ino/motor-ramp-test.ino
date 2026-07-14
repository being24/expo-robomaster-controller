#include <ArduinoJson.h>
#include <M5Unified.h>

#include <ESP32-TWAI-CAN.hpp>

namespace {

constexpr int CAN_TX = 2;
constexpr int CAN_RX = 1;
constexpr uint32_t MOTOR_COMMAND_ID = 0x1FF;
constexpr uint32_t MOTOR_FEEDBACK_ID = 0x205;

constexpr uint32_t COMMAND_INTERVAL_MS = 20;
constexpr uint32_t SERIAL_INTERVAL_MS = 100;
constexpr uint32_t FEEDBACK_TIMEOUT_MS = 500;

constexpr uint32_t INITIAL_STOP_MS = 3000;
constexpr uint32_t RAMP_UP_MS = 12000;
constexpr uint32_t HOLD_MS = 5000;
constexpr uint32_t RAMP_DOWN_MS = 12000;
constexpr uint32_t FINAL_STOP_MS = 5000;
constexpr uint32_t TEST_CYCLE_MS =
    INITIAL_STOP_MS + RAMP_UP_MS + HOLD_MS + RAMP_DOWN_MS + FINAL_STOP_MS;

constexpr float TEST_TARGET_RPM = 30.0f;

CanFrame rxFrame;
StaticJsonDocument<512> doc;

float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

float targetRpm = 0.0f;
float outputCurrent = 0.0f;
float currentRpm = 0.0f;
float instantRpm = 0.0f;
float angleDeg = 0.0f;
float motorCurrentA = 0.0f;
int16_t speedRpmRaw = 0;
int16_t torqueRaw = 0;
uint8_t motorTemperature = 0;

uint32_t feedbackCounter = 0;
uint32_t lastFeedbackMs = 0;
bool hasFeedback = false;

float previousAngle = 0.0f;
uint32_t previousAngleMs = 0;
bool angleInitialized = false;

constexpr size_t RPM_FILTER_SIZE = 5;
float rpmBuffer[RPM_FILTER_SIZE] = {};
size_t rpmBufferIndex = 0;
size_t rpmSampleCount = 0;

const float mappingCurrent[] = {
    0.0,  0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7,  0.8,  0.9,  1.0,  1.2,
    1.4,  1.6,  1.8,  2.0,  2.5,  3.0,  -0.1, -0.2, -0.3, -0.4, -0.5, -0.6,
    -0.7, -0.8, -0.9, -1.0, -1.2, -1.4, -1.6, -1.8, -2.0, -2.5, -3.0};

const float mappingRpm[] = {
    0.00,   0.00,   0.00,   0.00,   0.02,   0.08,   1.49,   4.68,  6.72,
    8.91,   11.65,  17.05,  22.74,  29.08,  34.89,  41.38,  56.11, 72.33,
    0.00,   -0.00,  -0.01,  -0.02,  -0.09,  -1.30,  -3.35,  -4.91, -7.42,
    -10.29, -14.45, -20.66, -25.99, -31.43, -37.67, -51.54, -66.49};

float rpmToCurrent(float rpm) {
  rpm = constrain(rpm, -66.49f, 72.33f);
  if (fabsf(rpm) < 0.1f) return 0.0f;

  const int begin = rpm > 0.0f ? 0 : 18;
  const int end = rpm > 0.0f ? 18 : 35;

  for (int i = begin; i < end; ++i) {
    const bool reached = rpm > 0.0f ? mappingRpm[i] >= rpm : mappingRpm[i] <= rpm;
    if (!reached) continue;

    if (i == begin || mappingRpm[i - 1] == mappingRpm[i]) {
      return mappingCurrent[i];
    }

    const float ratio =
        (rpm - mappingRpm[i - 1]) / (mappingRpm[i] - mappingRpm[i - 1]);
    return mappingCurrent[i - 1] +
           ratio * (mappingCurrent[i] - mappingCurrent[i - 1]);
  }

  return mappingCurrent[end - 1];
}

float calculateTargetRpm(uint32_t elapsedMs) {
  const uint32_t position = elapsedMs % TEST_CYCLE_MS;

  if (position < INITIAL_STOP_MS) return 0.0f;

  uint32_t phasePosition = position - INITIAL_STOP_MS;
  if (phasePosition < RAMP_UP_MS) {
    return TEST_TARGET_RPM * static_cast<float>(phasePosition) / RAMP_UP_MS;
  }

  phasePosition -= RAMP_UP_MS;
  if (phasePosition < HOLD_MS) return TEST_TARGET_RPM;

  phasePosition -= HOLD_MS;
  if (phasePosition < RAMP_DOWN_MS) {
    return TEST_TARGET_RPM *
           (1.0f - static_cast<float>(phasePosition) / RAMP_DOWN_MS);
  }

  return 0.0f;
}

void sendCurrent(float currentA) {
  constexpr float MOTOR_MAX_CURRENT_A = 3.0f;
  constexpr int16_t MOTOR_MAX_COMMAND = 16384;

  currentA = constrain(currentA, -MOTOR_MAX_CURRENT_A, MOTOR_MAX_CURRENT_A);
  int16_t command = static_cast<int16_t>(
      currentA * (static_cast<float>(MOTOR_MAX_COMMAND) / MOTOR_MAX_CURRENT_A));

  CanFrame frame = {};
  frame.identifier = MOTOR_COMMAND_ID;
  frame.extd = 0;
  frame.data_length_code = 8;
  frame.data[0] = static_cast<uint8_t>((command >> 8) & 0xFF);
  frame.data[1] = static_cast<uint8_t>(command & 0xFF);
  ESP32Can.writeFrame(frame);
}

float calculateInstantRpm(float currentAngle, uint32_t currentMs) {
  if (!angleInitialized) {
    previousAngle = currentAngle;
    previousAngleMs = currentMs;
    angleInitialized = true;
    return 0.0f;
  }

  const uint32_t elapsedMs = currentMs - previousAngleMs;
  if (elapsedMs == 0) return 0.0f;

  float angleDifference = currentAngle - previousAngle;
  if (angleDifference > 180.0f) angleDifference -= 360.0f;
  if (angleDifference < -180.0f) angleDifference += 360.0f;

  previousAngle = currentAngle;
  previousAngleMs = currentMs;
  return angleDifference * 1000.0f * 60.0f /
         (static_cast<float>(elapsedMs) * 360.0f);
}

float filterRpm(float rpm) {
  rpmBuffer[rpmBufferIndex] = rpm;
  rpmBufferIndex = (rpmBufferIndex + 1) % RPM_FILTER_SIZE;
  if (rpmSampleCount < RPM_FILTER_SIZE) ++rpmSampleCount;

  float sum = 0.0f;
  for (size_t i = 0; i < rpmSampleCount; ++i) sum += rpmBuffer[i];
  return rpmSampleCount > 0 ? sum / rpmSampleCount : 0.0f;
}

void readImu() {
  M5.update();
  if (!M5.Imu.update()) return;

  const auto data = M5.Imu.getImuData();
  accelX = data.accel.x;
  accelY = data.accel.y;
  accelZ = data.accel.z;
  gyroX = data.gyro.x;
  gyroY = data.gyro.y;
  gyroZ = data.gyro.z;
}

void readMotorFeedback() {
  while (ESP32Can.readFrame(rxFrame, 0)) {
    if (rxFrame.identifier != MOTOR_FEEDBACK_ID || rxFrame.data_length_code != 8) {
      continue;
    }

    const uint16_t mechanicalAngle =
        (static_cast<uint16_t>(rxFrame.data[0]) << 8) | rxFrame.data[1];
    speedRpmRaw = static_cast<int16_t>(
        (static_cast<uint16_t>(rxFrame.data[2]) << 8) | rxFrame.data[3]);
    torqueRaw = static_cast<int16_t>(
        (static_cast<uint16_t>(rxFrame.data[4]) << 8) | rxFrame.data[5]);
    motorTemperature = rxFrame.data[6];

    angleDeg = mechanicalAngle * 360.0f / 8192.0f;
    instantRpm = calculateInstantRpm(angleDeg, millis());
    currentRpm = filterRpm(instantRpm);
    motorCurrentA = torqueRaw / 2048.0f;

    lastFeedbackMs = millis();
    hasFeedback = true;
    ++feedbackCounter;
  }
}

void printTelemetry() {
  doc.clear();
  doc["motor"]["current"] = motorCurrentA;
  doc["motor"]["angle"] = angleDeg;
  doc["motor"]["speed"] = currentRpm;
  doc["motor"]["speed_raw"] = speedRpmRaw;
  doc["motor"]["speed_instant"] = instantRpm;
  doc["motor"]["torque"] = torqueRaw;
  doc["motor"]["temp"] = motorTemperature;

  doc["control"]["target_rpm"] = targetRpm;
  doc["control"]["current_rpm"] = currentRpm;
  doc["control"]["output_current"] = outputCurrent;
  doc["control"]["error"] = targetRpm - currentRpm;
  doc["control"]["has_feedback"] = hasFeedback;
  if (hasFeedback) {
    doc["control"]["feedback_age_ms"] = millis() - lastFeedbackMs;
  } else {
    doc["control"]["feedback_age_ms"] = nullptr;
  }

  doc["accel"]["x"] = roundf(accelX * 1000.0f) / 1000.0f;
  doc["accel"]["y"] = roundf(accelY * 1000.0f) / 1000.0f;
  doc["accel"]["z"] = roundf(accelZ * 1000.0f) / 1000.0f;

  doc["gyro"]["x"] = roundf(gyroX * 1000.0f) / 1000.0f;
  doc["gyro"]["y"] = roundf(gyroY * 1000.0f) / 1000.0f;
  doc["gyro"]["raw_z"] = roundf(gyroZ * 1000.0f) / 1000.0f;
  doc["gyro"]["z"] = roundf((-currentRpm * 6.0f) * 1000.0f) / 1000.0f;

  doc["timestamp"] = millis();
  doc["counter"] = feedbackCounter;

  serializeJson(doc, Serial);
  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) delay(10);

  auto config = M5.config();
  M5.begin(config);

  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(10);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  if (!ESP32Can.begin()) {
    Serial.println("{\"error\":\"CAN initialization failed\"}");
    while (true) delay(1000);
  }

  sendCurrent(0.0f);
}

void loop() {
  static uint32_t testStartMs = millis();
  static uint32_t lastCommandMs = 0;
  static uint32_t lastSerialMs = 0;

  const uint32_t now = millis();
  readImu();
  readMotorFeedback();

  if (hasFeedback && now - lastFeedbackMs > FEEDBACK_TIMEOUT_MS) {
    hasFeedback = false;
    angleInitialized = false;
    outputCurrent = 0.0f;
    sendCurrent(0.0f);
  }

  targetRpm = calculateTargetRpm(now - testStartMs);

  if (now - lastCommandMs >= COMMAND_INTERVAL_MS) {
    outputCurrent = hasFeedback ? rpmToCurrent(targetRpm) : 0.0f;
    sendCurrent(outputCurrent);
    lastCommandMs = now;
  }

  if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
    printTelemetry();
    lastSerialMs = now;
  }

  delay(1);
}
