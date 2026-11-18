// rc_link.h — ตัวรวมข้อความเป็นบรรทัด ใช้ร่วมกันทั้ง Bluetooth และ WiFi
// ข้อความทุกคำสั่งเป็นตัวอักษรธรรมดา จบด้วย '\n' เช่น  "M,80,-80\n"
#pragma once
#include <Arduino.h>

typedef void (*RcLineHandler)(const char* line);

class RcLineBuf {
public:
  // ป้อนข้อมูลดิบเข้ามา เจอ '\n' หรือ '\r' เมื่อไหร่ก็ส่งบรรทัดนั้นให้ cb
  void feed(const uint8_t* data, size_t n, RcLineHandler cb) {
    for (size_t i = 0; i < n; i++) {
      char c = (char)data[i];
      if (c == '\n' || c == '\r') {
        if (len && !skipping && cb) { buf[len] = 0; cb(buf); }
        len = 0;
        skipping = false;
      } else if (skipping) {
        // บรรทัดยาวเกิน ทิ้งจนกว่าจะขึ้นบรรทัดใหม่
      } else if (len < sizeof(buf) - 1) {
        buf[len++] = c;
      } else {
        len = 0;
        skipping = true;
      }
    }
  }
  // จบหนึ่งแพ็กเก็ตแล้ว ถ้ายังมีข้อความค้าง ให้ถือว่าจบบรรทัด
  void flush(RcLineHandler cb) {
    if (len && !skipping && cb) { buf[len] = 0; cb(buf); }
    len = 0;
    skipping = false;
  }
  void clear() { len = 0; skipping = false; }

private:
  char buf[48];
  size_t len = 0;
  bool skipping = false;
};
