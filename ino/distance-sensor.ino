#include <ArduinoJson.h>
#include <ETH.h>
#include <M5Unified.h>
#include <Network.h>
#include <NetworkUdp.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "wifi_config.h"

namespace {

// MINITOF-90 on the AtomS3 Grove port (Port A).
constexpr int TOF_SDA = 2;
constexpr int TOF_SCL = 1;
constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
constexpr uint32_t TOF_I2C_FREQUENCY_HZ = 400000;
constexpr uint32_t TOF_TIMEOUT_MS = 200;
constexpr uint32_t TOF_MEASUREMENT_INTERVAL_MS = 50;

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

constexpr uint32_t UDP_SEND_INTERVAL_MS = 100;
constexpr uint32_t UDP_BIND_RETRY_MS = 1000;
constexpr uint32_t DISPLAY_INTERVAL_MS = 200;
constexpr uint8_t DISPLAY_BRIGHTNESS = 48;

TwoWire tofWire(0);
VL53L0X tofSensor;
NetworkUDP udp;
StaticJsonDocument<256> telemetry;

volatile bool lanLinkUp = false;
volatile bool lanHasIp = false;
bool udpStarted = false;
IPAddress boundIp(0, 0, 0, 0);

uint16_t distanceMm = 0;
uint32_t sampleCount = 0;
uint32_t timeoutCount = 0;
uint32_t sendCount = 0;
uint32_t sendErrorCount = 0;
bool measurementValid = false;

bool isValidIp(const IPAddress& ip) {
  return ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0;
}

void onNetworkEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname("atoms3-tof");
      Serial.println("ETH started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      lanLinkUp = true;
      Serial.println("ETH link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      lanLinkUp = true;
      lanHasIp = true;
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
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
    Serial.printf("UDP sender ready: %s:%u -> %s:%u\n",
                  currentIp.toString().c_str(), TOF_UDP_SOURCE_PORT,
                  UDP_TARGET_IP.toString().c_str(), TOF_UDP_TARGET_PORT);
  } else {
    Serial.println("UDP sender bind failed");
  }
}

void buildTelemetry() {
  telemetry.clear();
  telemetry["sensor"] = "MINITOF-90";
  telemetry["distance_mm"] = distanceMm;
  telemetry["valid"] = measurementValid;
  telemetry["timeout"] = !measurementValid;
  telemetry["sample_count"] = sampleCount;
  telemetry["timeout_count"] = timeoutCount;
  telemetry["timestamp"] = millis();
  telemetry["counter"] = sendCount;
}

void sendTelemetry() {
  buildTelemetry();

  char buffer[256];
  const size_t length = serializeJson(telemetry, buffer, sizeof(buffer));
  serializeJson(telemetry, Serial);
  Serial.println();

  if (!udpStarted || length == 0 || length >= sizeof(buffer)) return;

  const bool packetStarted = udp.beginPacket(UDP_TARGET_IP, TOF_UDP_TARGET_PORT);
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
    Serial.println("UDP send failed");
  }
}

void updateDisplay() {
  M5.Display.startWrite();
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println("MINITOF-90 UDP");
  M5.Display.printf("IP: %s\n", ETH.localIP().toString().c_str());
  M5.Display.printf("LAN: %s / UDP: %s\n", lanLinkUp ? "UP" : "DOWN",
                    udpStarted ? "OK" : "WAIT");
  if (measurementValid) {
    M5.Display.printf("Distance: %u mm\n", distanceMm);
  } else {
    M5.Display.println("Distance: TIMEOUT");
  }
  M5.Display.printf("Sent: %lu Err: %lu\n",
                    static_cast<unsigned long>(sendCount),
                    static_cast<unsigned long>(sendErrorCount));
  M5.Display.endWrite();
}

[[noreturn]] void haltWithError(const char* serialMessage,
                                const char* displayMessage) {
  Serial.println(serialMessage);
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println(displayMessage);
  while (true) delay(1000);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) delay(10);

  auto config = M5.config();
  M5.begin(config);
  M5.Display.setRotation(3);
  M5.Display.setTextSize(1.0f);
  M5.Display.setBrightness(DISPLAY_BRIGHTNESS);

  tofWire.begin(TOF_SDA, TOF_SCL, TOF_I2C_FREQUENCY_HZ);
  tofWire.beginTransmission(TOF_I2C_ADDRESS);
  const uint8_t i2cError = tofWire.endTransmission();
  if (i2cError != 0) {
    Serial.printf("{\"error\":\"MINITOF-90 not found\",\"address\":\"0x29\","
                  "\"i2c_error\":%u}\n",
                  i2cError);
    haltWithError("TOF I2C probe failed", "TOF NOT FOUND");
  }

  tofSensor.setBus(&tofWire);
  tofSensor.setTimeout(TOF_TIMEOUT_MS);
  if (!tofSensor.init()) {
    haltWithError("MINITOF-90 initialization failed", "TOF INIT FAILED");
  }
  tofSensor.setMeasurementTimingBudget(50000);
  tofSensor.startContinuous(TOF_MEASUREMENT_INTERVAL_MS);

  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  SPI.setFrequency(10000000);
  Network.onEvent(onNetworkEvent);

  Serial.println("Starting Ethernet (W5500 over SPI)...");
  if (!ETH.begin(ETH_PHY_W5500, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ,
                 ETH_PHY_RST, SPI)) {
    haltWithError("ETH.begin() failed", "ETH INIT FAILED");
  }

  if (USE_STATIC_IP &&
      !ETH.config(TOF_LOCAL_IP, LOCAL_GATEWAY, LOCAL_SUBNET, LOCAL_DNS1,
                  LOCAL_DNS2)) {
    haltWithError("ETH.config() failed", "ETH CONFIG FAILED");
  }
}

void loop() {
  static uint32_t lastSendMs = 0;
  static uint32_t lastDisplayMs = 0;

  M5.update();
  syncUdpSocket();

  distanceMm = tofSensor.readRangeContinuousMillimeters();
  measurementValid = !tofSensor.timeoutOccurred();
  ++sampleCount;
  if (!measurementValid) ++timeoutCount;

  const uint32_t now = millis();
  if (now - lastSendMs >= UDP_SEND_INTERVAL_MS) {
    sendTelemetry();
    lastSendMs = now;
  }

  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    updateDisplay();
    lastDisplayMs = now;
  }
}
