#include <ArduinoJson.h>
#include <ETH.h>
#include <M5Unified.h>
#include <Network.h>
#include <NetworkUdp.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "wifi_config.h"

// Uncomment to enable status messages and JSON telemetry on USB serial.
// #define SERIAL_LOG_MODE

namespace {

// MINITOF-90 on the AtomS3 Grove port (Port A).
constexpr int TOF_SDA = 2;
constexpr int TOF_SCL = 1;
constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
constexpr uint32_t TOF_I2C_FREQUENCY_HZ = 400000;
constexpr uint32_t TOF_TIMEOUT_MS = 200;
constexpr uint32_t TOF_MEASUREMENT_TIMING_BUDGET_US = 20000;
constexpr uint32_t TOF_MEASUREMENT_INTERVAL_MS = 20;

// Atomic PoE V1.2.1 (W5500 over SPI) on AtomS3.
constexpr int ETH_SPI_SCK = 5;
constexpr int ETH_PHY_CS = 6;
constexpr int ETH_SPI_MISO = 7;
constexpr int ETH_SPI_MOSI = 8;
constexpr int ETH_PHY_IRQ = -1;
constexpr int ETH_PHY_RST = -1;
constexpr int ETH_PHY_ADDR = 1;

// Dedicated network identity and UDP ports for the ToF node.
// The motor node uses 192.168.1.30 and ports 9001/9002.
const IPAddress TOF_LOCAL_IP(192, 168, 1, 31);
constexpr uint16_t TOF_UDP_TARGET_PORT = 9011;
constexpr uint16_t TOF_UDP_SOURCE_PORT = 9012;

constexpr uint32_t UDP_SEND_INTERVAL_MS = 20;
constexpr uint32_t UDP_BIND_RETRY_MS = 1000;
constexpr uint32_t DISPLAY_INTERVAL_MS = 200;
constexpr uint8_t DISPLAY_BRIGHTNESS = 48;
constexpr uint32_t SENSOR_TASK_STACK_SIZE = 4096;
constexpr uint32_t NETWORK_TASK_STACK_SIZE = 4096;
constexpr uint32_t DISPLAY_TASK_STACK_SIZE = 4096;

TwoWire tofWire(0);
VL53L0X tofSensor;
NetworkUDP udp;
StaticJsonDocument<256> telemetry;

volatile bool lanLinkUp = false;
volatile bool lanHasIp = false;
volatile bool udpStarted = false;
IPAddress boundIp(0, 0, 0, 0);

struct MeasurementState {
  uint16_t distanceMm = 0;
  uint32_t sampleCount = 0;
  uint32_t timeoutCount = 0;
  bool valid = false;
};

portMUX_TYPE measurementMux = portMUX_INITIALIZER_UNLOCKED;
MeasurementState measurement;
volatile uint32_t sendCount = 0;
volatile uint32_t sendErrorCount = 0;

MeasurementState getMeasurementSnapshot() {
  portENTER_CRITICAL(&measurementMux);
  const MeasurementState snapshot = measurement;
  portEXIT_CRITICAL(&measurementMux);
  return snapshot;
}

bool isValidIp(const IPAddress& ip) {
  return ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0;
}

void onNetworkEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("atoms3-tof");
#ifdef SERIAL_LOG_MODE
      Serial.println("ETH started");
#endif
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      lanLinkUp = true;
#ifdef SERIAL_LOG_MODE
      Serial.println("ETH link up");
#endif
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      lanLinkUp = true;
      lanHasIp = true;
#ifdef SERIAL_LOG_MODE
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
#endif
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      lanHasIp = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      lanLinkUp = false;
      lanHasIp = false;
      break;
    default:
      break;
  }
}

void syncUdpSocket() {
  static uint32_t lastRetryMs = 0;
  const IPAddress currentIp = ETH.localIP();
  const bool ipReady = lanHasIp && isValidIp(currentIp);

  if (!ipReady) {
    if (udpStarted) udp.stop();
    udpStarted = false;
    boundIp = IPAddress(0, 0, 0, 0);
    return;
  }

  if (udpStarted && currentIp == boundIp) return;
  if (millis() - lastRetryMs < UDP_BIND_RETRY_MS) return;
  lastRetryMs = millis();

  if (udpStarted) udp.stop();
  udpStarted = udp.begin(TOF_UDP_SOURCE_PORT);
  if (udpStarted) {
    boundIp = currentIp;
#ifdef SERIAL_LOG_MODE
    Serial.printf("UDP sender ready: %s:%u -> %s:%u\n",
                  currentIp.toString().c_str(), TOF_UDP_SOURCE_PORT,
                  UDP_TARGET_IP.toString().c_str(), TOF_UDP_TARGET_PORT);
#endif
  } else {
#ifdef SERIAL_LOG_MODE
    Serial.println("UDP sender bind failed");
#endif
  }
}

void buildTelemetry(const MeasurementState& snapshot) {
  telemetry.clear();
  telemetry["sensor"] = "MINITOF-90";
  telemetry["distance_mm"] = snapshot.distanceMm;
  telemetry["valid"] = snapshot.valid;
  telemetry["timeout"] = !snapshot.valid;
  telemetry["sample_count"] = snapshot.sampleCount;
  telemetry["timeout_count"] = snapshot.timeoutCount;
  telemetry["timestamp"] = millis();
  telemetry["counter"] = sendCount;
}

void sendTelemetry() {
  buildTelemetry(getMeasurementSnapshot());

  char buffer[256];
  const size_t length = serializeJson(telemetry, buffer, sizeof(buffer));
#ifdef SERIAL_LOG_MODE
  serializeJson(telemetry, Serial);
  Serial.println();
#endif

  if (!udpStarted || length == 0 || length >= sizeof(buffer)) return;

  const bool packetStarted =
      udp.beginPacket(UDP_TARGET_IP, TOF_UDP_TARGET_PORT);
  bool ok = packetStarted;
  if (packetStarted) {
    ok = udp.write(reinterpret_cast<const uint8_t*>(buffer), length) == length;
    ok = udp.endPacket() && ok;
  }

  if (ok) {
    ++sendCount;
  } else {
    ++sendErrorCount;
    udp.stop();
    udpStarted = false;
#ifdef SERIAL_LOG_MODE
    Serial.println("UDP send failed");
#endif
  }
}

void updateDisplay() {
  const MeasurementState snapshot = getMeasurementSnapshot();
  const bool hasError =
      !snapshot.valid || !lanLinkUp || !lanHasIp || !udpStarted;

  M5.Display.startWrite();
  M5.Display.fillScreen(BLACK);

  if (!hasError) {
    // Normally show only the latest distance, large enough to read at a glance.
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextSize(5.0f);
    M5.Display.drawNumber(snapshot.distanceMm, M5.Display.width() / 2,
                          M5.Display.height() / 2 - 12);
    M5.Display.setTextSize(2.0f);
    M5.Display.drawString("mm", M5.Display.width() / 2,
                          M5.Display.height() - 18);
    M5.Display.endWrite();
    return;
  }

  // On an error (including startup/reconnection), show the detailed status.
  M5.Display.setTextDatum(top_left);
  M5.Display.setTextSize(1.0f);
  M5.Display.setCursor(0, 0);
  M5.Display.println("MINITOF-90 UDP");
  M5.Display.printf("IP: %s\n", ETH.localIP().toString().c_str());
  M5.Display.printf("LAN: %s / UDP: %s\n", lanLinkUp ? "UP" : "DOWN",
                    udpStarted ? "OK" : "WAIT");
  if (snapshot.valid) {
    M5.Display.printf("Distance: %u mm\n", snapshot.distanceMm);
  } else {
    M5.Display.println("Distance: TIMEOUT");
  }
  M5.Display.printf("Sent: %lu Err: %lu\n",
                    static_cast<unsigned long>(sendCount),
                    static_cast<unsigned long>(sendErrorCount));
  M5.Display.endWrite();
}

void sensorTask(void* parameter) {
  (void)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    const uint16_t distanceMm = tofSensor.readRangeContinuousMillimeters();
    const bool valid = !tofSensor.timeoutOccurred();

    portENTER_CRITICAL(&measurementMux);
    measurement.distanceMm = distanceMm;
    measurement.valid = valid;
    ++measurement.sampleCount;
    if (!valid) ++measurement.timeoutCount;
    portEXIT_CRITICAL(&measurementMux);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TOF_MEASUREMENT_INTERVAL_MS));
  }
}

void networkTask(void* parameter) {
  (void)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    syncUdpSocket();
    sendTelemetry();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(UDP_SEND_INTERVAL_MS));
  }
}

void displayTask(void* parameter) {
  (void)parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    M5.update();
    updateDisplay();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
  }
}

[[noreturn]] void haltWithError(const char* serialMessage,
                                const char* displayMessage) {
#ifdef SERIAL_LOG_MODE
  Serial.println(serialMessage);
#else
  (void)serialMessage;
#endif
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println(displayMessage);
  while (true) delay(1000);
}

}  // namespace

void setup() {
#ifdef SERIAL_LOG_MODE
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) delay(10);
#endif

  auto config = M5.config();
  M5.begin(config);
  M5.Display.setRotation(2);
  M5.Display.setTextSize(1.0f);
  M5.Display.setBrightness(DISPLAY_BRIGHTNESS);

  tofWire.begin(TOF_SDA, TOF_SCL, TOF_I2C_FREQUENCY_HZ);
  tofWire.beginTransmission(TOF_I2C_ADDRESS);
  const uint8_t i2cError = tofWire.endTransmission();
  if (i2cError != 0) {
#ifdef SERIAL_LOG_MODE
    Serial.printf(
        "{\"error\":\"MINITOF-90 not found\",\"address\":\"0x29\","
        "\"i2c_error\":%u}\n",
        i2cError);
#endif
    haltWithError("TOF I2C probe failed", "TOF NOT FOUND");
  }

  tofSensor.setBus(&tofWire);
  tofSensor.setTimeout(TOF_TIMEOUT_MS);
  if (!tofSensor.init()) {
    haltWithError("MINITOF-90 initialization failed", "TOF INIT FAILED");
  }
  tofSensor.setMeasurementTimingBudget(TOF_MEASUREMENT_TIMING_BUDGET_US);
  tofSensor.startContinuous(TOF_MEASUREMENT_INTERVAL_MS);

  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  SPI.setFrequency(10000000);
  Network.onEvent(onNetworkEvent);

#ifdef SERIAL_LOG_MODE
  Serial.println("Starting Ethernet (W5500 over SPI)...");
#endif
  if (!ETH.begin(ETH_PHY_W5500, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ,
                 ETH_PHY_RST, SPI)) {
    haltWithError("ETH.begin() failed", "ETH INIT FAILED");
  }

  if (USE_STATIC_IP && !ETH.config(TOF_LOCAL_IP, LOCAL_GATEWAY, LOCAL_SUBNET,
                                   LOCAL_DNS1, LOCAL_DNS2)) {
    haltWithError("ETH.config() failed", "ETH CONFIG FAILED");
  }

  const BaseType_t sensorTaskCreated = xTaskCreate(
      sensorTask, "ToFSensor", SENSOR_TASK_STACK_SIZE, nullptr, 3, nullptr);
  const BaseType_t networkTaskCreated = xTaskCreate(
      networkTask, "ToFNetwork", NETWORK_TASK_STACK_SIZE, nullptr, 2, nullptr);
  const BaseType_t displayTaskCreated = xTaskCreate(
      displayTask, "ToFDisplay", DISPLAY_TASK_STACK_SIZE, nullptr, 1, nullptr);

  if (sensorTaskCreated != pdPASS || networkTaskCreated != pdPASS ||
      displayTaskCreated != pdPASS) {
    haltWithError("FreeRTOS task creation failed", "TASK CREATE FAILED");
  }
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
