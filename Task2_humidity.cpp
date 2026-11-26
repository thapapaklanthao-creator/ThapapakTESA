#include <Preferences.h>

const char* TEAM_ID      = "TGR-11";
const bool  PRINT_QUOTES = true;

const int PIN_CHARGE = 25;
const int PIN_CMP    = 27;
const int PIN_BTN    = 26;
const int PIN_LED    = 2;

const double R1_OHM  = 1000000.0;
const double VCC_MV  = 3330.0;
const double VREF_MV = 2080.0;

// จุดปรับเทียบ RAW -> % ความชื้น (19 พ.ย.)
const double RAW_EMPTY_CAL = 22.31;
const double RAW_1 = 28.792;  const double M_1 = 6.9;
const double RAW_2 = 38.470;  const double M_2 = 11.3;
const double RAW_3 = 41.780;  const double M_3 = 14.5;

const bool CAL_MODE = false;   // true = ทดสอบ (แสดง RAW), false = ตอนส่งงาน

const unsigned long BOOT_WAIT_MS  = 3000;   // รอหลังเสียบไฟ ก่อนตั้งศูนย์อัตโนมัติ
const unsigned long MEAS_DELAY_MS = 700;    // รอหลังปล่อยปุ่ม ให้มือออกห่าง
const unsigned long TARE_HOLD_MS  = 5000;   // กดค้างนานเท่านี้ = บังคับตั้งศูนย์
const double MAX_OFFSET = 5.0;              // (เฉพาะอัตโนมัติ) offset เกินนี้ = กล่องไม่ว่าง

const int RUNS = 401;
const unsigned long DISCHARGE_US = 500;
uint32_t cycBuf[RUNS];
double lnFactor = 1.0;
double rawOffset = 0.0;
Preferences prefs;

uint32_t chargeOnce() {
  digitalWrite(PIN_CHARGE, LOW);
  delayMicroseconds(DISCHARGE_US);
  uint32_t start = ESP.getCycleCount();
  digitalWrite(PIN_CHARGE, HIGH);
  while (digitalRead(PIN_CMP) == HIGH) {
    if (ESP.getCycleCount() - start > 240UL * 20000) {
      digitalWrite(PIN_CHARGE, LOW);
      return 0;
    }
  }
  uint32_t cycles = ESP.getCycleCount() - start;
  digitalWrite(PIN_CHARGE, LOW);
  return cycles;
}

double measureRaw(int &okOut) {
  int ok = 0;
  for (int i = 0; i < RUNS; i++) {
    uint32_t c = chargeOnce();
    if (c > 0) cycBuf[ok++] = c;
  }
  okOut = ok;
  if (ok < RUNS / 2) return -1;

  for (int i = 1; i < ok; i++) {
    uint32_t k = cycBuf[i]; int j = i - 1;
    while (j >= 0 && cycBuf[j] > k) { cycBuf[j + 1] = cycBuf[j]; j--; }
    cycBuf[j + 1] = k;
  }
  int lo = ok / 4, hi = ok - ok / 4;
  uint64_t sum = 0;
  for (int i = lo; i < hi; i++) sum += cycBuf[i];
  double avgCycles = (double)sum / (double)(hi - lo);
  double us = avgCycles / (double)getCpuFrequencyMhz();
  return us / (R1_OHM * lnFactor) * 1e6;
}

double rawToMoisture(double raw) {
  double m;
  if (raw < RAW_2) m = M_1 + (M_2 - M_1) / (RAW_2 - RAW_1) * (raw - RAW_1);
  else             m = M_2 + (M_3 - M_2) / (RAW_3 - RAW_2) * (raw - RAW_2);
  if (m < 0)   m = 0;
  if (m > 100) m = 100;
  return m;
}

void blinkLed(int n, int ms) {
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED, HIGH); delay(ms);
    digitalWrite(PIN_LED, LOW);  delay(ms);
  }
}

// วัดกล่องว่างแล้วบันทึก offset
// force = true : บังคับบันทึกเสมอ (กดค้าง 5 วินาที)
// force = false: อัตโนมัติ มีกันพลาดถ้ากล่องไม่ว่าง
void tareNow(unsigned long waitMs, bool force) {
  delay(waitMs);                                    // รอมือออกห่าง
  double sum = 0; int n = 0;
  for (int k = 0; k < 5; k++) {
    int ok = 0;
    double r = measureRaw(ok);
    if (r > 0) { sum += r; n++; }
  }
  if (n == 0) { blinkLed(6, 80); return; }          // วัดไม่ได้ -> ใช้ค่าเดิม
  double newOffset = sum / n - RAW_EMPTY_CAL;
  if (!force && (newOffset > MAX_OFFSET || newOffset < -MAX_OFFSET)) {
    blinkLed(6, 80);                                // กล่องไม่ว่าง -> ใช้ค่าเดิม
    if (CAL_MODE) Serial.printf("\r\nTARE rejected, offset = %.3f (keep %.3f)\r\n", newOffset, rawOffset);
    return;
  }
  rawOffset = newOffset;
  prefs.putDouble("offset", rawOffset);
  blinkLed(3, 150);                                 // สำเร็จ
  if (CAL_MODE) Serial.printf("\r\nTARE done, offset = %.3f\r\n", rawOffset);
}

void doMeasurement(unsigned long tPress) {
  int ok = 0;
  double raw = measureRaw(ok);

  double rawCorr = raw - rawOffset;
  double m = (raw < 0) ? 0.0 : rawToMoisture(rawCorr);

  if (PRINT_QUOTES) Serial.printf("\"%s\"\r\n", TEAM_ID);
  else              Serial.printf("%s\r\n", TEAM_ID);
  Serial.printf("Moisture Level = %.2f %%\r\n", m);
  if (CAL_MODE) Serial.printf("RAW = %.3f  corr = %.3f  offset = %.3f  (ok %d/%d, %lu ms)\r\n",
                              raw, rawCorr, rawOffset, ok, RUNS, millis() - tPress);
  Serial.print("\r\n");
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_CHARGE, OUTPUT);
  digitalWrite(PIN_CHARGE, LOW);
  pinMode(PIN_CMP, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  double ratio = VREF_MV / VCC_MV;
  if (ratio > 0.05 && ratio < 0.95) lnFactor = -log(1.0 - ratio);

  prefs.begin("tgr11", false);
  rawOffset = prefs.getDouble("offset", 0.0);       // ค่าเดิมสำรองไว้

  tareNow(BOOT_WAIT_MS, false);                     // ตั้งศูนย์อัตโนมัติทุกครั้งที่เสียบไฟ

  if (CAL_MODE) Serial.printf("\r\nReady - offset = %.3f\r\n", rawOffset);
}

void loop() {
  static int last = HIGH;
  int b = digitalRead(PIN_BTN);
  if (last == HIGH && b == LOW) {
    unsigned long tPress = millis();
    digitalWrite(PIN_LED, HIGH);                     // LED 200 ms ทันทีที่กด
    while (millis() - tPress < 200) {}
    digitalWrite(PIN_LED, LOW);

    bool longHold = false;
    while (digitalRead(PIN_BTN) == LOW) {            // รอปล่อยปุ่ม
      if (!longHold && millis() - tPress >= TARE_HOLD_MS) {
        longHold = true;
        digitalWrite(PIN_LED, HIGH);                 // ไฟติดค้าง = ปล่อยได้แล้ว
      }
      delay(5);
    }

    if (longHold) {
      digitalWrite(PIN_LED, LOW);
      tareNow(2000, true);                           // บังคับตั้งศูนย์ ไม่พิมพ์ผล
    } else {
      delay(MEAS_DELAY_MS);                          // รอมือออกห่าง
      doMeasurement(tPress);
    }

    delay(150);
    b = HIGH;
  }
  last = b;
}
