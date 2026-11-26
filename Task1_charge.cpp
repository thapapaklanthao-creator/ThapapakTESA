const char* TEAM_ID      = "TGR-11";
const bool  PRINT_QUOTES = true;

const int PIN_CHARGE = 25;
const int PIN_CMP    = 27;
const int PIN_BTN    = 26;
const int PIN_LED    = 2;

const double R1_OHM  = 1000000.0;
const double VCC_MV  = 3330.0;
const double VREF_MV = 2080.0;

double CAL_A = 1.0292;
double CAL_B = -11.824;
const bool CAL_MODE = false;         // false = แสดงผลตามกติกาเท่านั้น

const int RUNS = 401;
const unsigned long DISCHARGE_US = 500;
uint32_t cycBuf[RUNS];
double lnFactor = 1.0;

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

void doMeasurement() {
  unsigned long t0 = millis();
  digitalWrite(PIN_LED, HIGH);

  int ok = 0;
  double raw = measureRaw(ok);

  while (millis() - t0 < 200) {}
  digitalWrite(PIN_LED, LOW);

  double c = (raw < 0) ? 0.0 : CAL_A * raw + CAL_B;
  if (c < 0) c = 0;

  if (PRINT_QUOTES) Serial.printf("\"%s\"\r\n", TEAM_ID);
  else              Serial.printf("%s\r\n", TEAM_ID);
  Serial.printf("Capacitance = %.2f pF\r\n", c);
  if (CAL_MODE) Serial.printf("RAW = %.3f  (ok %d/%d, %lu ms)\r\n", raw, ok, RUNS, millis() - t0);
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

  if (CAL_MODE) Serial.print("\r\nReady - press the button\r\n");
}

void loop() {
  static int last = HIGH;
  int b = digitalRead(PIN_BTN);
  if (last == HIGH && b == LOW) {
    doMeasurement();
    while (digitalRead(PIN_BTN) == LOW) delay(5);
    delay(150);
    b = HIGH;
  }
  last = b;
}
