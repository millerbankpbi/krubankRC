// rc_wifi.h — เว็บเซิร์ฟเวอร์ + WebSocket แบบเล็กสำหรับ UNO R4 WiFi (WiFiS3)
//   GET /     -> ส่งหน้าจอย (บีบอัด gzip ไว้ใน rc_page.h)
//   GET /ws   -> เปิดท่อ WebSocket รับคำสั่งแบบเรียลไทม์
// เชื่อมได้ทีละ 1 จอย ถ้ามีเครื่องใหม่ต่อเข้ามา เครื่องเก่าจะถูกตัด
#pragma once
#include <Arduino.h>
#include <WiFiS3.h>
#pragma GCC diagnostic ignored "-Wdeprecated-copy"   // WiFiClient ของ WiFiS3 คัดลอกได้ปกติ แค่คอมไพเลอร์เตือน
#include "rc_link.h"

class RcWebServer {
public:
  typedef void (*Hook)();
  typedef void (*ConnHook)(bool connected);

  void setPage(const uint8_t* gz, size_t len) { page = gz; pageLen = len; }
  void setHandler(RcLineHandler h) { handler = h; }
  void setConnHook(ConnHook h) { connHook = h; }
  void setBusyHook(Hook h) { busyHook = h; }   // เรียกก่อนงานที่บล็อกนาน (เช่น ส่งหน้าเว็บ)

  void begin() { server.begin(); }
  bool wsConnected() const { return wsOpen; }
  uint32_t pagesServed() const { return pages; }

  // เรียกทุกรอบ loop()
  void poll() {
    uint32_t now = millis();
    if (wsOpen) readWs(now);
    // ตรวจหาเครื่องที่ต่อเข้ามาใหม่ (ถ้ากำลังบังคับอยู่ ตรวจห่างขึ้นเพื่อไม่ให้หน่วง)
    if (now - lastAccept >= (wsOpen ? ACCEPT_MS_BUSY : ACCEPT_MS_IDLE)) {
      lastAccept = now;
      WiFiClient c = server.accept();
      if (c) addPending(c, now);
      pollPending(now);
    }
    now = millis();                          // อ่านเวลาใหม่ เพราะด้านบนอาจเพิ่งเปิดท่อ (lastRx ใหม่กว่า now เดิม)
    if (wsOpen && now - lastAliveCheck >= 1000) {
      lastAliveCheck = now;
      if (now - lastRx > WS_IDLE_MS || !ws.connected()) closeWs(false);
    }
  }

  // ส่งข้อความกลับไปที่จอย (1 บรรทัด = 1 เฟรม)
  bool send(const char* text) {
    if (!wsOpen) return false;
    size_t n = strlen(text);
    if (n > 120) n = 120;
    uint8_t f[124];
    f[0] = 0x81;               // FIN + text
    f[1] = (uint8_t)n;         // เซิร์ฟเวอร์ไม่ต้อง mask
    memcpy(f + 2, text, n);
    return ws.write(f, n + 2) == n + 2;
  }

  void closeWs(bool sendClose) {
    if (!wsOpen) return;
    if (sendClose) { uint8_t f[2] = { 0x88, 0x00 }; ws.write(f, 2); }
    ws.stop();
    wsOpen = false;
    rx.clear();
    frameLen = 0;
    if (connHook) connHook(false);
  }

private:
  static const uint16_t ACCEPT_MS_IDLE = 40;
  static const uint16_t ACCEPT_MS_BUSY = 200;
  static const uint16_t WS_IDLE_MS = 4000;     // จอยส่ง ping ทุก 1 วินาที ถ้าเงียบเกินนี้ถือว่าหลุด
  static const uint16_t HEADER_TIMEOUT = 3000;  // ms รอหัวคำขอ HTTP

  WiFiServer server{ 80 };
  WiFiClient ws;
  bool wsOpen = false;
  RcLineBuf rx;
  RcLineHandler handler = nullptr;
  ConnHook connHook = nullptr;
  Hook busyHook = nullptr;
  const uint8_t* page = nullptr;
  size_t pageLen = 0;
  uint32_t pages = 0;
  uint32_t lastAccept = 0, lastAliveCheck = 0, lastRx = 0;
  uint8_t frame[160];
  size_t frameLen = 0;

  // ---------- HTTP ----------
  // เครื่องที่ต่อเข้ามาแล้วแต่หัวคำขอยังมาไม่ครบ อ่านทีละนิดไม่ให้ loop ค้าง
  struct Pending {
    WiFiClient c;
    bool used = false;
    uint32_t t0 = 0;
    char line[100];
    uint8_t len = 0;
    uint8_t lineNo = 0;
    bool upgrade = false;
    char path[24];
    char key[32];
  };
  Pending pend[2];

  void addPending(WiFiClient& c, uint32_t now) {
    Pending* slot = nullptr;
    for (auto& p : pend) if (!p.used) { slot = &p; break; }
    if (!slot) {                             // เต็ม: ไล่อันที่รอนานสุดออก
      slot = pend[0].t0 <= pend[1].t0 ? &pend[0] : &pend[1];
      slot->c.stop();
    }
    slot->c = c;
    slot->used = true;
    slot->t0 = now;
    slot->len = slot->lineNo = 0;
    slot->upgrade = false;
    slot->path[0] = slot->key[0] = 0;
  }

  void pollPending(uint32_t now) {
    for (auto& p : pend) {
      if (!p.used) continue;
      if (now - p.t0 > HEADER_TIMEOUT) { p.c.stop(); p.used = false; continue; }
      int avail = p.c.available();
      if (avail <= 0) continue;
      uint8_t tmp[64];
      while (avail > 0 && p.used) {
        int r = p.c.read(tmp, avail < (int)sizeof(tmp) ? avail : (int)sizeof(tmp));
        if (r <= 0) break;
        avail -= r;
        for (int i = 0; i < r && p.used; i++) {
          int st = feedHeader(p, (char)tmp[i]);
          if (st < 0) { p.c.stop(); p.used = false; }
          else if (st > 0) { p.used = false; respond(p); }
        }
      }
    }
  }

  // คืนค่า 1 = หัวคำขอครบ, 0 = ยังไม่ครบ, -1 = ผิดรูปแบบ
  int feedHeader(Pending& p, char ch) {
    if (ch == '\r') return 0;
    if (ch != '\n') {
      if (p.len < sizeof(p.line) - 1) p.line[p.len++] = ch;
      return 0;
    }
    p.line[p.len] = 0;
    if (p.len == 0) return p.lineNo > 0 ? 1 : -1;   // บรรทัดว่าง = จบหัว
    if (p.lineNo == 0) {
      const char* sp = strchr(p.line, ' ');          // "GET /path HTTP/1.1"
      if (!sp) return -1;
      const char* s = sp + 1;
      size_t k = 0;
      while (*s && *s != ' ' && k < sizeof(p.path) - 1) p.path[k++] = *s++;
      p.path[k] = 0;
    } else if (startsWithI(p.line, "sec-websocket-key:")) {
      copyValue(p.line + 18, p.key, sizeof(p.key));
    } else if (startsWithI(p.line, "upgrade:")) {
      char v[16];
      copyValue(p.line + 8, v, sizeof(v));
      p.upgrade = startsWithI(v, "websocket");
    }
    p.lineNo++;
    p.len = 0;
    return 0;
  }

  void respond(Pending& p) {
    WiFiClient& c = p.c;
    const char* path = p.path;
    if (strcmp(path, "/ws") == 0 && p.upgrade && p.key[0]) {
      if (wsOpen) closeWs(true);           // เครื่องใหม่มาแทน
      char acc[32];
      wsAccept(p.key, acc);
      char resp[160];
      int n = snprintf(resp, sizeof(resp),
                       "HTTP/1.1 101 Switching Protocols\r\n"
                       "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                       "Sec-WebSocket-Accept: %s\r\n\r\n", acc);
      c.write((const uint8_t*)resp, n);
      ws = c;
      wsOpen = true;
      frameLen = 0;
      rx.clear();
      lastRx = lastAliveCheck = millis();
      if (connHook) connHook(true);
      return;
    }

    if (busyHook) busyHook();
    if ((strcmp(path, "/") == 0 || strncmp(path, "/index", 6) == 0 || path[1] == '?') && page) {
      char hdr[200];
      int n = snprintf(hdr, sizeof(hdr),
                       "HTTP/1.1 200 OK\r\n"
                       "Content-Type: text/html; charset=utf-8\r\n"
                       "Content-Encoding: gzip\r\n"
                       "Content-Length: %u\r\n"
                       "Cache-Control: no-cache\r\n"
                       "Connection: close\r\n\r\n", (unsigned)pageLen);
      c.write((const uint8_t*)hdr, n);
      const size_t CHUNK = 1024;
      uint8_t tmp[CHUNK];
      for (size_t off = 0; off < pageLen; off += CHUNK) {
        size_t k = pageLen - off < CHUNK ? pageLen - off : CHUNK;
        memcpy(tmp, page + off, k);          // คัดลอกจาก flash ก่อนส่ง
        if (c.write(tmp, k) != k) break;
      }
      pages++;
    } else if (strcmp(path, "/favicon.ico") == 0) {
      const char* r = "HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n";
      c.write((const uint8_t*)r, strlen(r));
    } else {
      const char* r = "HTTP/1.1 302 Found\r\nLocation: /\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
      c.write((const uint8_t*)r, strlen(r));
    }
    delay(2);
    c.stop();
  }

  static bool startsWithI(const char* s, const char* prefix) {
    while (*prefix) {
      char a = *s++, b = *prefix++;
      if (a >= 'A' && a <= 'Z') a += 32;
      if (a != b) return false;
    }
    return true;
  }
  static void copyValue(const char* s, char* out, size_t sz) {
    while (*s == ' ') s++;
    size_t k = 0;
    while (*s && *s != ' ' && k < sz - 1) out[k++] = *s++;
    out[k] = 0;
  }

  // ---------- WebSocket ----------
  void readWs(uint32_t now) {
    int avail = ws.available();
    if (avail <= 0) return;
    size_t room = sizeof(frame) - frameLen;
    if ((size_t)avail > room) avail = (int)room;
    int r = ws.read(frame + frameLen, avail);
    if (r <= 0) return;
    frameLen += r;
    lastRx = now;
    parseFrames();
  }

  void parseFrames() {
    while (frameLen >= 2) {
      uint8_t op = frame[0] & 0x0F;
      bool fin = frame[0] & 0x80;
      bool masked = frame[1] & 0x80;
      size_t plen = frame[1] & 0x7F;
      size_t hdr = 2;
      if (plen == 126) {
        if (frameLen < 4) return;
        plen = ((size_t)frame[2] << 8) | frame[3];
        hdr = 4;
      } else if (plen == 127) {
        closeWs(true);                       // เฟรมใหญ่เกินไป ไม่รองรับ
        return;
      }
      if (masked) hdr += 4;
      if (hdr + plen > sizeof(frame)) { closeWs(true); return; }
      if (frameLen < hdr + plen) return;     // รอข้อมูลเพิ่ม

      uint8_t* pay = frame + hdr;
      if (masked) {
        const uint8_t* m = pay - 4;
        for (size_t i = 0; i < plen; i++) pay[i] ^= m[i & 3];
      }
      if (op == 0x1 || op == 0x0 || op == 0x2) {
        rx.feed(pay, plen, handler);
        if (fin) rx.flush(handler);
      } else if (op == 0x8) {                // ปิด
        closeWs(true);
        return;
      } else if (op == 0x9) {                // ping -> pong
        uint8_t f[2 + 125];
        f[0] = 0x8A;
        f[1] = (uint8_t)plen;
        memcpy(f + 2, pay, plen);
        ws.write(f, plen + 2);
      }
      size_t used = hdr + plen;
      memmove(frame, frame + used, frameLen - used);
      frameLen -= used;
      if (!wsOpen) return;
    }
  }

  // Sec-WebSocket-Accept = base64( SHA1(key + GUID) )
  static void wsAccept(const char* key, char* out) {
    static const char GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    Sha1 s;
    s.update((const uint8_t*)key, strlen(key));
    s.update((const uint8_t*)GUID, sizeof(GUID) - 1);
    uint8_t d[20];
    s.final(d);
    base64(d, 20, out);
  }

  static void base64(const uint8_t* in, size_t n, char* out) {
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
      uint32_t v = (uint32_t)in[i] << 16;
      if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
      if (i + 2 < n) v |= in[i + 2];
      out[o++] = T[(v >> 18) & 63];
      out[o++] = T[(v >> 12) & 63];
      out[o++] = i + 1 < n ? T[(v >> 6) & 63] : '=';
      out[o++] = i + 2 < n ? T[v & 63] : '=';
    }
    out[o] = 0;
  }

  // SHA-1 ขนาดเล็ก (ใช้แค่ตอนจับมือ WebSocket)
  struct Sha1 {
    uint32_t h[5] = { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0 };
    uint8_t blk[64];
    size_t blen = 0;
    uint64_t total = 0;
    static uint32_t rol(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }
    void block() {
      uint32_t w[80];
      for (int i = 0; i < 16; i++)
        w[i] = ((uint32_t)blk[i * 4] << 24) | ((uint32_t)blk[i * 4 + 1] << 16) | ((uint32_t)blk[i * 4 + 2] << 8) | blk[i * 4 + 3];
      for (int i = 16; i < 80; i++) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
      uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
      for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) { f = (b & c) | (~b & d); k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6; }
        uint32_t t = rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol(b, 30); b = a; a = t;
      }
      h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }
    void update(const uint8_t* p, size_t n) {
      total += n;
      while (n--) {
        blk[blen++] = *p++;
        if (blen == 64) { block(); blen = 0; }
      }
    }
    void final(uint8_t out[20]) {
      uint64_t bits = total * 8;
      uint8_t pad = 0x80;
      update(&pad, 1);
      uint8_t z = 0;
      while (blen != 56) update(&z, 1);
      for (int i = 7; i >= 0; i--) { uint8_t b = (uint8_t)(bits >> (i * 8)); update(&b, 1); }
      for (int i = 0; i < 5; i++) {
        out[i * 4] = h[i] >> 24; out[i * 4 + 1] = h[i] >> 16;
        out[i * 4 + 2] = h[i] >> 8; out[i * 4 + 3] = h[i];
      }
    }
  };
};
