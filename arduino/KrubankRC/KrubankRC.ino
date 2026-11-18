/* =====================================================================
   KrubankRC — บังคับหุ่น MATRIX Mini R4 แบบเรียลไทม์ ผ่าน WiFi หรือ Bluetooth
   ---------------------------------------------------------------------
   เปิดเครื่อง → จอขึ้นเมนูให้เลือก (รอ 2.5 วินาที)
       กด UP   = WiFi       หุ่นต่อ hotspot มือถือ แล้วโชว์ IP บนจอ
                            มือถือเปิด Chrome พิมพ์ IP นั้น (เช่น 172.20.10.3) ได้หน้าจอยทันที
       กด DOWN = Bluetooth  เปิดไฟล์ KrubankRC.html จากลิงก์ https (iPhone ใช้แอป Bluefy)
                            แล้วกดปุ่มเชื่อมต่อ เลือก KRUBANKR4
       ไม่กด    = ใช้ค่า BOOT_DEFAULT

   คำสั่งที่จอยส่งมา (ตัวอักษร 1 บรรทัด):
       M,ซ้าย,ขวา   กำลังล้อ -100..100  (ส่งซ้ำทุก ~0.1 วิ ตอนกดค้าง)
       B,ข้อความ     ปุ่มพิเศษ → ไปทำงานใน onButton() แท็บ Buttons
       P,n          ping วัดความหน่วง (หุ่นตอบ p,n)
       X            หยุดฉุกเฉิน
       H            ขอข้อมูลหุ่น (หุ่นตอบ I,ชื่อ,โหมด)
   ความปลอดภัย: ถ้าไม่ได้รับคำสั่ง M เกิน DRIVE_TIMEOUT_MS หุ่นหยุดเอง (สัญญาณหลุดก็ไม่วิ่งหนี)
   ===================================================================== */
#define ENABLE_DRIVEDC_BRAKE_DELAY
#include <MatrixMiniR4.h>

// ---------- เปิด/ปิดช่องทาง (ตั้ง 0 เพื่อตัดออก ประหยัดหน่วยความจำ) ----------
#define USE_WIFI 1
#define USE_BLE 1

#if USE_WIFI
#include <WiFiS3.h>
#include "rc_wifi.h"
#include "rc_page.h"
#endif
#if USE_BLE
#include "rc_ble.h"
#endif
#include "rc_servo.h"

// ================= ตั้งค่า =================
const char* ROBOT_NAME = "KRUBANKR4";   // ชื่อ Bluetooth และชื่อ WiFi ตอนหุ่นปล่อยเอง

// WiFi ผ่าน hotspot มือถือ
const char* WIFI_SSID = "KRUBANK";      // ชื่อ hotspot ของมือถือ (ตรงตัวพิมพ์เล็กใหญ่)  ใส่ "" = ไม่ต่อ hotspot ปล่อย WiFi เองเลย
const char* WIFI_PASS = "12345678";     // รหัส hotspot
const bool WIFI_AP_FALLBACK = true;     // หา hotspot ไม่เจอ → หุ่นปล่อย WiFi ชื่อ KRUBANKR4 เอง
const char* AP_PASS = "12345678";       // รหัส WiFi ที่หุ่นปล่อยเอง (อย่างน้อย 8 ตัว)

const uint8_t LINK_NONE = 0, LINK_WIFI = 1, LINK_BLE = 2;
const uint8_t BOOT_DEFAULT = LINK_WIFI; // ไม่กดปุ่มตอนเปิดเครื่อง ใช้ช่องทางนี้
const uint16_t BOOT_WAIT_MS = 2500;

// การขับ
const bool LEFT_REVERSE = false, RIGHT_REVERSE = true;  // เหมือน DriveDC.begin(1, 2, false, true)
const int MAX_POWER = 100;              // กำลังสูงสุดที่หุ่นยอมวิ่ง (จอยส่ง 100 = ค่านี้)
const int ACCEL_PER_S = 800;            // เร่งได้กี่ % ต่อวินาที (800 = 0→100 ใน 0.13 วิ) 0 = ทันที
const int DECEL_PER_S = 2500;           // ผ่อน/เบรกได้กี่ % ต่อวินาที                       0 = ทันที
const uint16_t DRIVE_TIMEOUT_MS = 400;  // ไม่ได้รับคำสั่งเกินนี้ = หยุด
const bool BRAKE_ON_STOP = true;        // ปล่อยจอยแล้วเบรก (false = ปล่อยไหล)
const uint8_t MOTOR_MIN_MS = 20;        // ส่งคำสั่งมอเตอร์ถี่สุดทุก 20 ms
const uint16_t MOTOR_REFRESH_MS = 250;  // วิ่งค้างอยู่ ส่งซ้ำทุก 250 ms กันมอเตอร์หลุดคำสั่ง
const uint16_t SERVO_DEFAULT_MS = 300;  // คำสั่ง S2=90 ไม่ระบุเวลา ใช้เวลานี้

const bool DEBUG_SERIAL = false;        // true = พิมพ์คำสั่งที่ได้รับออก Serial Monitor
// ===========================================

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

uint8_t linkMode = LINK_NONE;
bool linkUp = false;
#if USE_WIFI
RcWebServer* web = nullptr;
bool wifiAP = false;
IPAddress myIP;
uint32_t lastWifiCheck = 0;
#endif
#if USE_BLE
RcBle ble;
#endif
RcServos servos;

int tgtL = 0, tgtR = 0;          // ค่าที่จอยสั่ง
float outL = 0, outR = 0;        // ค่าที่ส่งให้มอเตอร์จริง (หลังผ่านการเร่ง/ผ่อน)
int sentL = 9999, sentR = 9999;
uint32_t lastMoveCmd = 0, lastMotorSend = 0, lastDriveTick = 0;
uint32_t cmdCount = 0, lastAckMs = 0, lastBatt = 0, lastOled = 0;
float battV = 0;
char lastBtn[18] = "-";
char lastAck[18] = "";
int8_t ledState = -1;

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2);
  MiniR4.DriveDC.begin(1, 2, LEFT_REVERSE, RIGHT_REVERSE);
  MiniR4.DriveDC.brake(true);
  MiniR4.OLED.setTextSize(1);
  MiniR4.OLED.setTextColor(SSD1306_WHITE);

  servos.setWriter(servoWriteHW);
  setupServos();                 // อยู่ในแท็บ Buttons

  linkMode = chooseLink();
#if USE_WIFI
  if (linkMode == LINK_WIFI) startWiFi();
#endif
#if USE_BLE
  if (linkMode == LINK_BLE) startBLE();
#endif
  lastDriveTick = millis();
  showStatus();
}

void loop() {
#if USE_WIFI
  if (linkMode == LINK_WIFI && web) web->poll();
#endif
#if USE_BLE
  if (linkMode == LINK_BLE) ble.poll();
#endif
  driveUpdate();
  servos.update();
  periodic();
}

// ===================== รับคำสั่ง =====================
void onLine(const char* s) {
  if (DEBUG_SERIAL) Serial.println(s);
  switch (s[0]) {
    case 'M': {                                   // M,ซ้าย,ขวา
      const char* p = strchr(s, ',');
      if (!p) return;
      char* e;
      long l = strtol(p + 1, &e, 10);
      if (*e != ',') return;
      long r = strtol(e + 1, nullptr, 10);
      setTarget((int)l, (int)r);
      break;
    }
    case 'P': {                                   // P,n -> p,n
      char out[24];
      snprintf(out, sizeof(out), "p%s\n", s + 1);
      linkSend(out);
      break;
    }
    case 'B':
      if (s[1] == ',') handleButton(s + 2);
      break;
    case 'X':
      stopNow();
      break;
    case 'H':
      sendHello();
      break;
  }
}

void linkSend(const char* s) {
#if USE_WIFI
  if (linkMode == LINK_WIFI && web) web->send(s);
#endif
#if USE_BLE
  if (linkMode == LINK_BLE) ble.send(s);
#endif
}

void onLink(bool up) {
  linkUp = up;
  stopNow();                                      // ต่อใหม่/หลุด ให้หยุดก่อนเสมอ
  if (up) sendHello();
  lastOled = 0;                                   // อัปเดตจอทันที
}

void sendHello() {
  char out[24];
  snprintf(out, sizeof(out), "I,%s,%s\n", ROBOT_NAME, linkMode == LINK_BLE ? "BLE" : "WIFI");
  linkSend(out);
  lastBatt = 0;                                   // ส่งแบตตามไปเลย
}

void handleButton(const char* cmd) {
  bool ok = onButton(cmd) || builtinCommand(cmd);
  strncpy(lastBtn, cmd, sizeof(lastBtn) - 1);
  lastBtn[sizeof(lastBtn) - 1] = 0;
  // ตอบกลับให้จอยรู้ว่าได้รับแล้ว (ปุ่มกดค้างส่งซ้ำ จะไม่ตอบถี่เกิน)
  uint32_t now = millis();
  if (strcmp(lastAck, cmd) != 0 || now - lastAckMs > 400) {
    char out[22];
    snprintf(out, sizeof(out), "L,%s%.15s\n", ok ? "" : "?", cmd);
    linkSend(out);
    strncpy(lastAck, cmd, sizeof(lastAck) - 1);
    lastAck[sizeof(lastAck) - 1] = 0;
    lastAckMs = now;
  }
}

bool is(const char* a, const char* b) { return strcasecmp(a, b) == 0; }

// คำสั่งสำเร็จรูป: S2=90  S2=90@500  S2+5  S2-5  BEEP  BEEP=1500,200  LED=r,g,b  STOP
bool builtinCommand(const char* c) {
  if ((c[0] == 'S' || c[0] == 's') && c[1] >= '1' && c[1] <= '4') {
    uint8_t ch = c[1] - '0';
    char* e;
    if (c[2] == '=') {
      long a = strtol(c + 3, &e, 10);
      long ms = (*e == '@') ? strtol(e + 1, nullptr, 10) : SERVO_DEFAULT_MS;
      servos.moveTo(ch, (int)a, (uint16_t)ms);
      return true;
    }
    if (c[2] == '+' || c[2] == '-') {
      long d = strtol(c + 3, &e, 10);
      servos.nudge(ch, c[2] == '+' ? (int)d : -(int)d, 80);
      return true;
    }
    return false;
  }
  if (strncasecmp(c, "BEEP", 4) == 0) {
    long f = 1500, ms = 120;
    if (c[4] == '=') {
      char* e;
      f = strtol(c + 5, &e, 10);
      if (*e == ',') ms = strtol(e + 1, nullptr, 10);
    }
    MiniR4.Buzzer.Tone((uint16_t)f, (uint32_t)ms);
    return true;
  }
  if (strncasecmp(c, "LED=", 4) == 0) {
    char* e;
    long r = strtol(c + 4, &e, 10);
    long g = (*e == ',') ? strtol(e + 1, &e, 10) : 0;
    long b = (*e == ',') ? strtol(e + 1, &e, 10) : 0;
    MiniR4.LED.setColor(1, r, g, b);
    MiniR4.LED.setColor(2, r, g, b);
    ledState = -1;
    return true;
  }
  if (is(c, "STOP")) { stopNow(); return true; }
  return false;
}

void servoWriteHW(uint8_t ch, int a) {
  switch (ch) {
    case 1: MiniR4.RC1.setAngle(a); break;
    case 2: MiniR4.RC2.setAngle(a); break;
    case 3: MiniR4.RC3.setAngle(a); break;
    case 4: MiniR4.RC4.setAngle(a); break;
  }
}

// ===================== ขับมอเตอร์ =====================
void setTarget(int l, int r) {
  l = constrain(l, -100, 100);
  r = constrain(r, -100, 100);
  tgtL = l * MAX_POWER / 100;
  tgtR = r * MAX_POWER / 100;
  lastMoveCmd = millis();
  cmdCount++;
}

void stopNow() {
  tgtL = tgtR = 0;
  outL = outR = 0;
  MiniR4.DriveDC.brake(true);
  sentL = sentR = 0;
  lastMotorSend = millis();
}

float slew(float cur, int tgt, float dt) {
  bool speedingUp = fabsf((float)tgt) > fabsf(cur) && (float)tgt * cur >= 0;
  int rate = speedingUp ? ACCEL_PER_S : DECEL_PER_S;
  if (rate <= 0) return tgt;
  float step = rate * dt;
  float d = tgt - cur;
  if (d > step) return cur + step;
  if (d < -step) return cur - step;
  return tgt;
}

void driveUpdate() {
  uint32_t now = millis();
  if ((tgtL || tgtR) && now - lastMoveCmd > DRIVE_TIMEOUT_MS) tgtL = tgtR = 0;   // สัญญาณหาย → หยุด

  float dt = (now - lastDriveTick) / 1000.0f;
  lastDriveTick = now;
  if (dt > 0.1f) dt = 0.1f;
  outL = slew(outL, tgtL, dt);
  outR = slew(outR, tgtR, dt);
  int L = (int)lroundf(outL), R = (int)lroundf(outR);

  if (L == 0 && R == 0) {
    if (sentL != 0 || sentR != 0) {
      if (BRAKE_ON_STOP) MiniR4.DriveDC.brake(true);
      else MiniR4.DriveDC.Move(0, 0);
      sentL = sentR = 0;
      lastMotorSend = now;
    }
    return;
  }
  bool changed = (L != sentL || R != sentR) && now - lastMotorSend >= MOTOR_MIN_MS;
  if (changed || now - lastMotorSend >= MOTOR_REFRESH_MS) {
    MiniR4.DriveDC.Move(L, R);
    sentL = L;
    sentR = R;
    lastMotorSend = now;
  }
}

// ===================== งานเป็นระยะ (แบต จอ ไฟ) =====================
void periodic() {
  uint32_t now = millis();
  bool driving = tgtL || tgtR || sentL || sentR;

  if (linkUp && now - lastBatt >= 2000) {
    lastBatt = now;
    battV = MiniR4.PWR.getBattVoltage();
    char out[16];
    snprintf(out, sizeof(out), "V,%d.%02d\n", (int)battV, (int)(battV * 100) % 100);
    linkSend(out);
  }

  int8_t want = linkUp ? 1 : (linkMode == LINK_BLE ? 2 : 3);
  if (want != ledState) {
    ledState = want;
    uint8_t r = 0, g = 0, b = 0;
    if (want == 1) g = 60;            // เขียว = จอยต่ออยู่
    else if (want == 2) { r = 30; b = 60; }  // ม่วง = รอ Bluetooth
    else b = 60;                      // ฟ้า = รอ WiFi
    MiniR4.LED.setColor(1, r, g, b);
    MiniR4.LED.setColor(2, r, g, b);
  }

  // จอ OLED ใช้เวลาเขียนนาน ตอนกำลังวิ่งจึงอัปเดตห่างขึ้น ให้หุ่นตอบสนองเร็วที่สุด
  if (now - lastOled >= (driving ? 1500u : 300u)) {
    lastOled = now;
    showStatus();
  }

#if USE_WIFI
  // hotspot หลุด → ต่อใหม่ (เฉพาะตอนไม่มีจอยต่ออยู่)
  if (linkMode == LINK_WIFI && !wifiAP && !linkUp && now - lastWifiCheck >= 3000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      oled4("WiFi lost", "reconnect to", WIFI_SSID, "");
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      myIP = WiFi.localIP();
      lastWifiCheck = millis();
    }
  }
#endif
}

void oled4(const char* a, const char* b, const char* c, const char* d) {
  MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(0, 0);  MiniR4.OLED.print(a);
  MiniR4.OLED.setCursor(0, 8);  MiniR4.OLED.print(b);
  MiniR4.OLED.setCursor(0, 16); MiniR4.OLED.print(c);
  MiniR4.OLED.setCursor(0, 24); MiniR4.OLED.print(d);
  MiniR4.OLED.display();
}

void showStatus() {
  char l1[24], l2[24], l3[24], l4[24];
#if USE_WIFI
  if (linkMode == LINK_WIFI) {
    snprintf(l1, sizeof(l1), "%s %d.%d.%d.%d", wifiAP ? "AP" : "WiFi", myIP[0], myIP[1], myIP[2], myIP[3]);
    if (wifiAP) snprintf(l2, sizeof(l2), "%s/%s", ROBOT_NAME, AP_PASS);
    else snprintf(l2, sizeof(l2), "%s", linkUp ? "joy connected" : "open IP in Chrome");
  }
#endif
#if USE_BLE
  if (linkMode == LINK_BLE) {
    snprintf(l1, sizeof(l1), "BLE %s", ROBOT_NAME);
    snprintf(l2, sizeof(l2), "%s", linkUp ? "joy connected" : "waiting for joy");
  }
#endif
  if (linkMode == LINK_NONE) { snprintf(l1, sizeof(l1), "no link"); l2[0] = 0; }
  snprintf(l3, sizeof(l3), "L%4d R%4d #%lu", sentL == 9999 ? 0 : sentL, sentR == 9999 ? 0 : sentR, (unsigned long)(cmdCount % 100000));
  snprintf(l4, sizeof(l4), "%s %d.%dV", lastBtn, (int)battV, (int)(battV * 10) % 10);
  oled4(l1, l2, l3, l4);
}

// ===================== เลือกช่องทางตอนเปิดเครื่อง =====================
uint8_t chooseLink() {
#if USE_WIFI && USE_BLE
  uint32_t t0 = millis(), lastDraw = 0;
  while (millis() - t0 < BOOT_WAIT_MS) {
    if (MiniR4.BTN_UP.getState()) { waitRelease(); return LINK_WIFI; }
    if (MiniR4.BTN_DOWN.getState()) { waitRelease(); return LINK_BLE; }
    if (millis() - lastDraw >= 250) {
      lastDraw = millis();
      char d[24];
      snprintf(d, sizeof(d), "auto %s in %lus", BOOT_DEFAULT == LINK_WIFI ? "WiFi" : "BLE",
               (unsigned long)((BOOT_WAIT_MS - (millis() - t0)) / 1000 + 1));
      oled4("KrubankRC", "UP   = WiFi", "DOWN = Bluetooth", d);
    }
    delay(10);
  }
  return BOOT_DEFAULT;
#elif USE_WIFI
  return LINK_WIFI;
#else
  return LINK_BLE;
#endif
}

void waitRelease() {
  delay(30);
  while (MiniR4.BTN_UP.getState() || MiniR4.BTN_DOWN.getState()) delay(10);
}

#if USE_WIFI
void startWiFi() {
  if (WiFi.status() == WL_NO_MODULE) {
    oled4("WiFi module", "not found", "update firmware", "");
    while (true) delay(1000);
  }
  bool ok = false;
  if (WIFI_SSID[0]) {
    oled4("WiFi: joining", WIFI_SSID, "please wait...", "");
    WiFi.setTimeout(8000);
    for (int i = 0; i < 2 && !ok; i++) ok = WiFi.begin(WIFI_SSID, WIFI_PASS) == WL_CONNECTED;
  }
  if (ok) {
    uint32_t t0 = millis();
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0) && millis() - t0 < 5000) delay(100);
  } else if (WIFI_AP_FALLBACK || !WIFI_SSID[0]) {
    oled4(WIFI_SSID[0] ? "hotspot not found" : "own WiFi mode", "start own WiFi:", ROBOT_NAME, AP_PASS);
    WiFi.beginAP(ROBOT_NAME, AP_PASS);
    wifiAP = true;
    delay(1000);
  }
  myIP = WiFi.localIP();

  web = new RcWebServer();
  web->setPage(RC_PAGE_GZ, RC_PAGE_GZ_LEN);
  web->setHandler(onLine);
  web->setConnHook(onLink);
  web->setBusyHook(stopNow);
  web->begin();
}
#endif

#if USE_BLE
void startBLE() {
  ble.setHandler(onLine);
  ble.setConnHook(onLink);
  if (!ble.begin(ROBOT_NAME)) {
    oled4("Bluetooth", "start failed", "update firmware", "");
    while (true) delay(1000);
  }
}
#endif
