// rc_ble.h — Bluetooth LE แบบ Nordic UART (NUS) สำหรับ UNO R4 WiFi (ArduinoBLE)
//   RX 6E400002 : มือถือเขียนคำสั่งมา (รองรับ write without response = เร็วสุด)
//   TX 6E400003 : บอร์ดส่งข้อความกลับ (notify)
// ข้อความแต่ละก้อนต้องไม่เกิน 20 ไบต์ (ขนาดมาตรฐานของ BLE)
#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "rc_link.h"

namespace rcble {
  BLEService uart("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
  BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLERead | BLENotify, 20);
  RcLineBuf rx;
  RcLineHandler handler = nullptr;
  void (*connHook)(bool) = nullptr;
  bool linked = false;

  void onWritten(BLEDevice, BLECharacteristic c) {
    // เรียกทุกครั้งที่มีการเขียน จึงไม่ตกหล่นแม้มาหลายก้อนติดกัน
    rx.feed(c.value(), c.valueLength(), handler);
    rx.flush(handler);
  }
  void onConnect(BLEDevice) {
    linked = true;
    rx.clear();
    BLE.stopAdvertise();
    if (connHook) connHook(true);
  }
  void onDisconnect(BLEDevice) {
    linked = false;
    if (connHook) connHook(false);
    BLE.advertise();
  }
}

class RcBle {
public:
  void setHandler(RcLineHandler h) { rcble::handler = h; }
  void setConnHook(void (*h)(bool)) { rcble::connHook = h; }

  bool begin(const char* name) {
    if (!BLE.begin()) return false;
    BLE.setLocalName(name);
    BLE.setDeviceName(name);
    BLE.setAdvertisedService(rcble::uart);
    rcble::uart.addCharacteristic(rcble::rxChar);
    rcble::uart.addCharacteristic(rcble::txChar);
    BLE.addService(rcble::uart);
    rcble::rxChar.setEventHandler(BLEWritten, rcble::onWritten);
    BLE.setEventHandler(BLEConnected, rcble::onConnect);
    BLE.setEventHandler(BLEDisconnected, rcble::onDisconnect);
    BLE.setConnectionInterval(6, 12);   // ขอช่วงสื่อสาร 7.5–15 ms (มือถือเป็นคนตัดสินสุดท้าย)
    BLE.advertise();
    return true;
  }

  void poll() { BLE.poll(); }
  bool connected() const { return rcble::linked; }

  bool send(const char* text) {
    if (!rcble::linked || !rcble::txChar.subscribed()) return false;
    size_t n = strlen(text);
    if (n > 20) n = 20;
    return rcble::txChar.writeValue((const uint8_t*)text, (int)n) > 0;
  }
};
