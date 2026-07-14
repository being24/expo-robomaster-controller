#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <IPAddress.h>

// LAN & UDP 設定
// ご自身の環境に合わせて変更してください

// 送信先のIPアドレス
static IPAddress UDP_TARGET_IP(192, 168, 1, 7);

// AtomS3側の固定IP設定
const bool USE_STATIC_IP = true;
static IPAddress LOCAL_IP(192, 168, 1, 30);
static IPAddress LOCAL_GATEWAY(192, 168, 1, 1);
static IPAddress LOCAL_SUBNET(255, 255, 255, 0);
static IPAddress LOCAL_DNS1(192, 168, 1, 1);
static IPAddress LOCAL_DNS2(8, 8, 8, 8);

const uint16_t POST_UDP_PORT = 9001;     // 送信ポート（UDP）
const uint16_t RECEIVE_UDP_PORT = 9000;  // 受信ポート（UDP）
const uint16_t SEND_UDP_PORT = 9002;     // AtomS3側送信元ポート

#endif