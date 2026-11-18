// =====================================================================
//  Buttons — ตั้งว่า "ปุ่มบนจอย" สั่งหุ่นทำอะไร
// ---------------------------------------------------------------------
//  ในหน้าเว็บ แต่ละปุ่มตั้ง "ข้อความที่ส่ง" ได้เอง (กดฟันเฟือง → ปุ่ม)
//  กดปุ่มแล้วหุ่นได้ "B,KEEP" → มาเรียก onButton("KEEP") ข้างล่างนี้
//
//  คำสั่งสำเร็จรูป ใช้ได้เลยไม่ต้องเขียนโค้ด (พิมพ์ในช่องข้อความของปุ่ม):
//    S2=90        servo ช่อง RC2 ไปที่ 90 องศา (ค่อย ๆ ไปใน 0.3 วิ)
//    S2=90@800    ไปที่ 90 องศา ภายใน 800 ms
//    S2+5  S2-5   เพิ่ม/ลด 5 องศา  (ตั้งปุ่มเป็น "กดค้างส่งซ้ำ" แขนจะขยับต่อเนื่อง)
//    BEEP         เสียงบี๊บ     BEEP=2000,300  (ความถี่ Hz, เวลา ms)
//    LED=0,80,0   ไฟ RGB บนบอร์ด
//    STOP         หยุดล้อและแขนทันที
//  ข้อความยาวได้ไม่เกิน 16 ตัวอักษร
// =====================================================================

// ----- มุม servo แขนคีบ (ค่าเดียวกับ Gripper.ino) -----
const int ARM_UP = 170, ARM_DOWN = 51;            // RC2 แขน
const int CLAW3_OPEN = 153, CLAW3_CLOSE = 62;     // RC3 ก้ามซ้าย
const int CLAW4_OPEN = 49, CLAW4_CLOSE = 131;     // RC4 ก้ามขวา

// ----- ท่าชุด: { RC1, RC2, RC3, RC4, เวลาเคลื่อน ms, รอหลังจบ ms }   -1 = ไม่ขยับตัวนั้น -----
const ServoStep KEEP_STEPS[] = {
  { -1, -1,       CLAW3_OPEN,  CLAW4_OPEN,  150, 0   },   // กางก้ามรอ
  { -1, ARM_DOWN, -1,          -1,          900, 0   },   // ค่อย ๆ ลดแขนลง
  { -1, -1,       CLAW3_CLOSE, CLAW4_CLOSE, 350, 150 },   // บีบ แล้วรอให้แน่น
  { -1, ARM_UP,   -1,          -1,          600, 0   },   // ยกขึ้น
};
const ServoStep DROP_STEPS[] = {
  { -1, ARM_DOWN, -1,          -1,          900, 100 },   // ค่อย ๆ วางลง
  { -1, -1,       CLAW3_OPEN,  CLAW4_OPEN,  500, 0   },   // ค่อย ๆ กางปล่อย
};

// เรียกครั้งเดียวตอนเปิดเครื่อง
void setupServos() {
  // servos.setLimit(2, 45, 175);     // กันแขนชน: RC2 หมุนได้แค่ 45..175 องศา
  servos.jump(2, ARM_UP);             // ยกแขนขึ้น
  servos.jump(3, CLAW3_OPEN);         // กางก้าม
  servos.jump(4, CLAW4_OPEN);
}

// คืนค่า true = รู้จักคำสั่งนี้ (จอยจะขึ้นว่าได้รับแล้ว)
bool onButton(const char* cmd) {
  if (is(cmd, "KEEP")) { servos.run(KEEP_STEPS, COUNT(KEEP_STEPS)); return true; }   // คีบ
  if (is(cmd, "DROP")) { servos.run(DROP_STEPS, COUNT(DROP_STEPS)); return true; }   // วาง

  if (is(cmd, "OPEN")) {                                    // กางก้าม
    servos.moveTo(3, CLAW3_OPEN, 300);
    servos.moveTo(4, CLAW4_OPEN, 300);
    return true;
  }
  if (is(cmd, "CLOSE")) {                                   // หนีบ
    servos.moveTo(3, CLAW3_CLOSE, 300);
    servos.moveTo(4, CLAW4_CLOSE, 300);
    return true;
  }
  if (is(cmd, "ARMUP"))   { servos.moveTo(2, ARM_UP, 600);   return true; }
  if (is(cmd, "ARMDOWN")) { servos.moveTo(2, ARM_DOWN, 900); return true; }

  // ---- เพิ่มคำสั่งของครูเองตรงนี้ ----
  // ตัวอย่างปุ่มที่ตั้ง "ตอนกด" = TURBO และ "ตอนปล่อย" = TURBO_OFF
  // if (is(cmd, "TURBO"))     { MiniR4.LED.setColor(1, 80, 0, 0); return true; }
  // if (is(cmd, "TURBO_OFF")) { MiniR4.LED.setColor(1, 0, 60, 0); return true; }

  return false;   // ไม่รู้จัก → ลองคำสั่งสำเร็จรูป (S2=90, BEEP, ...)
}
