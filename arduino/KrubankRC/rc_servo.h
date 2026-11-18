// rc_servo.h — ขยับ servo แบบนุ่ม ๆ โดยไม่หยุดโปรแกรม (ไม่ใช้ delay)
// ระหว่างแขนกำลังเคลื่อน หุ่นยังรับคำสั่งวิ่งจากจอยได้ตามปกติ
#pragma once
#include <Arduino.h>

// หนึ่งขั้นของท่าชุด: มุมเป้าหมายของ servo 1–4 (-1 = ไม่ขยับตัวนั้น), เวลาเคลื่อน, เวลารอหลังจบ
struct ServoStep {
  int16_t a1, a2, a3, a4;
  uint16_t ms;
  uint16_t waitMs;
};

class RcServos {
public:
  typedef void (*Writer)(uint8_t ch, int angle);

  void setWriter(Writer w) { writer = w; }
  void setLimit(uint8_t ch, int lo, int hi) { if (ch >= 1 && ch <= 4) { minA[ch] = lo; maxA[ch] = hi; } }

  // กระโดดไปทันที (ใช้ตอนเริ่มต้น)
  void jump(uint8_t ch, int angle) {
    if (ch < 1 || ch > 4) return;
    angle = clampA(ch, angle);
    pos[ch] = from[ch] = target[ch] = angle;
    moving[ch] = false;
    write(ch, angle, true);
  }

  // ค่อย ๆ ไปที่ angle ภายใน ms มิลลิวินาที (ms = 0 คือไปทันที)
  void moveTo(uint8_t ch, int angle, uint16_t ms) {
    if (ch < 1 || ch > 4) return;
    angle = clampA(ch, angle);
    if (!known[ch] || ms == 0) { jump(ch, angle); return; }
    from[ch] = pos[ch];
    target[ch] = angle;
    t0[ch] = millis();
    dur[ch] = ms;
    moving[ch] = true;
  }

  // ขยับเพิ่ม/ลดจากเป้าหมายเดิม (ใช้กับปุ่มกดค้าง)
  void nudge(uint8_t ch, int delta, uint16_t ms) {
    if (ch < 1 || ch > 4) return;
    int base = known[ch] ? (int)target[ch] : 90;
    moveTo(ch, base + delta, ms);
  }

  int angleOf(uint8_t ch) const { return (ch >= 1 && ch <= 4) ? (int)(pos[ch] + 0.5f) : -1; }
  int targetOf(uint8_t ch) const { return (ch >= 1 && ch <= 4) ? target[ch] : -1; }
  bool busy() const { return macroLen > 0 || moving[1] || moving[2] || moving[3] || moving[4]; }

  // เล่นท่าชุด (เช่น คีบ / วาง) ทีละขั้น ไม่บล็อกโปรแกรม
  void run(const ServoStep* steps, uint8_t n) {
    macro = steps;
    macroLen = n;
    macroIdx = 0;
    startStep();
  }
  void stopAll() {
    macroLen = 0;
    for (uint8_t ch = 1; ch <= 4; ch++) { moving[ch] = false; target[ch] = (int)(pos[ch] + 0.5f); }
  }

  // เรียกทุกรอบ loop()
  void update() {
    uint32_t now = millis();
    for (uint8_t ch = 1; ch <= 4; ch++) {
      if (!moving[ch]) continue;
      uint32_t el = now - t0[ch];
      if (el >= dur[ch]) {
        pos[ch] = target[ch];
        moving[ch] = false;
        write(ch, target[ch], true);
      } else {
        float t = (float)el / dur[ch];
        float e = 0.5f - 0.5f * cosf(PI * t);          // ออกตัวช้า กลางเร็ว จบช้า
        pos[ch] = from[ch] + (target[ch] - from[ch]) * e;
        if (now - lastWrite[ch] >= WRITE_MS) write(ch, (int)(pos[ch] + 0.5f), false);
      }
    }
    if (macroLen) {
      const ServoStep& s = macro[macroIdx];
      if (now - stepT0 >= (uint32_t)s.ms + s.waitMs) {
        macroIdx++;
        if (macroIdx >= macroLen) macroLen = 0;
        else startStep();
      }
    }
  }

private:
  static const uint8_t WRITE_MS = 15;     // ส่งมุมใหม่ให้ servo ถี่สุดทุก 15 ms
  Writer writer = nullptr;
  float pos[5] = { 0 }, from[5] = { 0 };
  int16_t target[5] = { 0 }, written[5] = { -1, -1, -1, -1, -1 };
  int16_t minA[5] = { 0, 0, 0, 0, 0 }, maxA[5] = { 180, 180, 180, 180, 180 };
  bool known[5] = { false }, moving[5] = { false };
  uint32_t t0[5] = { 0 }, lastWrite[5] = { 0 };
  uint16_t dur[5] = { 0 };
  const ServoStep* macro = nullptr;
  uint8_t macroLen = 0, macroIdx = 0;
  uint32_t stepT0 = 0;

  int clampA(uint8_t ch, int a) const { return a < minA[ch] ? minA[ch] : (a > maxA[ch] ? maxA[ch] : a); }

  void write(uint8_t ch, int a, bool force) {
    known[ch] = true;
    if (!force && a == written[ch]) return;
    written[ch] = a;
    lastWrite[ch] = millis();
    if (writer) writer(ch, a);
  }

  void startStep() {
    const ServoStep& s = macro[macroIdx];
    stepT0 = millis();
    const int16_t a[5] = { -1, s.a1, s.a2, s.a3, s.a4 };
    for (uint8_t ch = 1; ch <= 4; ch++)
      if (a[ch] >= 0) moveTo(ch, a[ch], s.ms);
  }
};
