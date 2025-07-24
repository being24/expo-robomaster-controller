#include <SPI.h>

#include "esp_system.h"  // ← 追加
#include "mcp2515_can.h"

#define DUMMY_CS_PIN 33
#define BUTTON_PIN 37  // Aボタン（本体左）

mcp2515_can CAN(DUMMY_CS_PIN);

const int16_t MAX_VOLTAGE = 10000;
const int ACC_TIME = 8000;
const int DEC_TIME = 20000;
const int STOP_TIME = 10000;
const int PERIOD = ACC_TIME + DEC_TIME + STOP_TIME;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(BUTTON_PIN, INPUT);

  SPI.begin(25, 0, 26, DUMMY_CS_PIN);

  Serial.println("Starting CAN initialization...");

  while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_16MHz)) {
    Serial.println("CAN init fail, retry...");
    delay(100);
  }

  Serial.println("CAN BUS Shield init ok!");
  Serial.println("in_voltage,angle,speed,torque,temp");
}

void loop() {
  // Aボタン（GPIO37）が押されたらソフトリセット
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Button pressed → Restarting...");
    delay(500);     // 押しっぱなし対策のため少し待つ
    esp_restart();  // ← ソフトウェアリセット
  }

  Serial.println("Checking CAN messages...");

  unsigned long now = millis();
  unsigned long t = now % PERIOD;
  int16_t voltage = 0;

  if (t < ACC_TIME) {
    voltage = MAX_VOLTAGE * t / ACC_TIME;
  } else if (t < ACC_TIME + DEC_TIME) {
    voltage = MAX_VOLTAGE * (ACC_TIME + DEC_TIME - t) / DEC_TIME;
  }

  static unsigned long lastSend = 0;
  if (now - lastSend > 20) {
    lastSend = now;
    byte data[8] = {0};
    data[0] = (voltage >> 8) & 0xFF;
    data[1] = voltage & 0xFF;
    CAN.sendMsgBuf(0x1FF, 0, 8, data);
  }

  unsigned char len = 0;
  unsigned char rxBuf[8];
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&len, rxBuf);
    long unsigned int rxId = CAN.getCanId();
    if (rxId == 0x205) {
      uint16_t mech_angle = (rxBuf[0] << 8) | rxBuf[1];
      int16_t speed = (rxBuf[2] << 8) | rxBuf[3];
      int16_t torque = (rxBuf[4] << 8) | rxBuf[5];
      uint8_t temp = rxBuf[6];
      Serial.print(voltage);
      Serial.print(",");
      Serial.print(mech_angle);
      Serial.print(",");
      Serial.print(speed);
      Serial.print(",");
      Serial.print(torque);
      Serial.print(",");
      Serial.println(temp);
    }
  }
}
