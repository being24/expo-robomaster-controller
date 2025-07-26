#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi & UDP 設定
// ご自身の環境に合わせて変更してください
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";
const char* UDP_TARGET_IP = "192.168.1.100";  // 送信先のIPアドレス
const uint16_t POST_UDP_PORT = 8888;          // 送信ポート（UDP）
const uint16_t RECEIVE_UDP_PORT = 8887;       // 受信ポート（UDP）

#endif
