#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi & UDP 設定
// ご自身の環境に合わせて変更してください
const char* WIFI_SSID = "瞳Phone15";
const char* WIFI_PASSWORD = "airiphone";
const char* UDP_TARGET_IP = "172.20.10.3";  // 送信先のIPアドレス
const uint16_t POST_UDP_PORT = 9001;          // 送信ポート（UDP）
const uint16_t RECEIVE_UDP_PORT = 9000;       // 受信ポート（UDP）

#endif
