#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <Preferences.h>
#include "driver/ledc.h"

// =======================
// MODBUS RS485
// =======================
#define SLAVE_ID   3        // <-- GANTI: 1/2/3 sesuai omni
#define RS485_DE   4
#define RS485_RX   16
#define RS485_TX   17

ModbusRTUSlave modbus(Serial2, RS485_DE);

// =======================
// HOLDING REGISTERS
// =======================
static constexpr uint8_t NUM_HREG = 16;
uint16_t HREG[NUM_HREG];

// Register map
static constexpr uint16_t REG_CMD      = 0;
static constexpr uint16_t REG_PROX     = 1;
static constexpr uint16_t REG_MODE     = 2;
static constexpr uint16_t REG_SCALE    = 3;
static constexpr uint16_t REG_PWM1     = 4;
static constexpr uint16_t REG_PWM2     = 5;
static constexpr uint16_t REG_PWM3     = 6;
static constexpr uint16_t REG_PWM4     = 7;
static constexpr uint16_t REG_STOPMODE = 8;
static constexpr uint16_t REG_BRAKEMS  = 9;

// Command codes
static constexpr uint16_t CMD_STOP        = 0;
static constexpr uint16_t CMD_MAJU        = 1;
static constexpr uint16_t CMD_MUNDUR      = 2;
static constexpr uint16_t CMD_KANAN       = 3;
static constexpr uint16_t CMD_KIRI        = 4;
static constexpr uint16_t CMD_KANAN_ATAS  = 5;
static constexpr uint16_t CMD_KANAN_BAWAH = 6;
static constexpr uint16_t CMD_KIRI_ATAS   = 7;
static constexpr uint16_t CMD_KIRI_BAWAH  = 8;

// stop mode
static constexpr uint16_t STOP_COAST = 0;
static constexpr uint16_t STOP_BRAKE = 1;

// =======================
// PROXIMITY (aktif-LOW)
// =======================
static constexpr uint8_t  PIN_PROX = 23;
static constexpr uint16_t PROX_UPDATE_MS     = 2;
static constexpr uint8_t  PROX_STABLE_COUNT  = 1;

// =======================
// PWM / MOTOR
// =======================
static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t  PWM_RES  = 8; // 0..255

// Motor pins (MX1508 = 2 pin per motor)
const uint8_t PIN_M1_A1 = 27; const uint8_t PIN_M1_A2 = 14;
const uint8_t PIN_M2_A3 = 12; const uint8_t PIN_M2_A4 = 13;
const uint8_t PIN_M3_A1 = 26; const uint8_t PIN_M3_A2 = 25;
const uint8_t PIN_M4_A3 = 33; const uint8_t PIN_M4_A4 = 32;

// LED indikator (opsional) - hati-hati bootstrap pin 5 & 15
#define LED_ON  LOW
#define LED_OFF HIGH
const uint8_t L_MAJU   = 19;
const uint8_t L_KANAN  = 18;
const uint8_t L_MUNDUR = 5;   // bootstrap pin
const uint8_t L_KIRI   = 15;  // bootstrap pin

// LEDC channels (2 per motor)
enum {
  CH_M1_FWD = 0, CH_M1_REV,
  CH_M2_FWD,     CH_M2_REV,
  CH_M3_FWD,     CH_M3_REV,
  CH_M4_FWD,     CH_M4_REV
};

struct MotorHW {
  uint8_t pinFwd; uint8_t pinRev;
  uint8_t chFwd;  uint8_t chRev;
};

// mapping "maju" sama seperti kode i2c sebelumnya
MotorHW M[4] = {
  {PIN_M1_A2, PIN_M1_A1, CH_M1_FWD, CH_M1_REV},
  {PIN_M2_A3, PIN_M2_A4, CH_M2_FWD, CH_M2_REV},
  {PIN_M3_A2, PIN_M3_A1, CH_M3_FWD, CH_M3_REV},
  {PIN_M4_A3, PIN_M4_A4, CH_M4_FWD, CH_M4_REV}
};

// polaritas tambahan per motor bila perlu (+1 / -1)
int8_t POL[4] = {+1, +1, +1, +1};

static inline uint8_t clamp255(int v) {
  return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static inline int scaleDuty(int duty, uint16_t scale255) {
  // duty: -255..255, scale255: 0..255
  int sign = (duty >= 0) ? 1 : -1;
  int mag  = abs(duty);
  int out  = (mag * (int)scale255) / 255;
  return sign * out;
}

// =======================
// NVS (Preferences)
// =======================
Preferences prefs;

// PWM kalibrasi per motor (persist)
uint8_t p1 = 212, p2 = 255, p3 = 210, p4 = 208;

// caching untuk deteksi perubahan register
uint16_t lastCmd = 0xFFFF;
uint16_t lastScale = 255;
uint16_t lastStopMode = STOP_COAST;
uint16_t lastBrakeMs = 120;

uint8_t  lastP1, lastP2, lastP3, lastP4;

// save debouncing (biar flash nggak sering ditulis)
bool nvsDirty = false;
uint32_t nvsDirtySinceMs = 0;
static constexpr uint32_t NVS_SAVE_DELAY_MS = 800; // tunggu stabil sebelum commit

void loadPWMfromNVS() {
  prefs.begin("pwm", true);
  p1 = prefs.getUChar("pwm1", p1);
  p2 = prefs.getUChar("pwm2", p2);
  p3 = prefs.getUChar("pwm3", p3);
  p4 = prefs.getUChar("pwm4", p4);
  prefs.end();
}

void commitPWMtoNVS() {
  prefs.begin("pwm", false);
  prefs.putUChar("pwm1", p1);
  prefs.putUChar("pwm2", p2);
  prefs.putUChar("pwm3", p3);
  prefs.putUChar("pwm4", p4);
  prefs.end();
}

// =======================
// LED + MOTOR primitives
// =======================
void setLED(bool maju, bool kanan, bool kiri, bool mundur) {
  digitalWrite(L_MAJU,   maju   ? LED_ON : LED_OFF);
  digitalWrite(L_KANAN,  kanan  ? LED_ON : LED_OFF);
  digitalWrite(L_KIRI,   kiri   ? LED_ON : LED_OFF);
  digitalWrite(L_MUNDUR, mundur ? LED_ON : LED_OFF);
}

void motorWrite(uint8_t idx, int speed) {
  speed *= POL[idx];
  const uint8_t fwdDuty = speed > 0 ? clamp255(speed) : 0;
  const uint8_t revDuty = speed < 0 ? clamp255(-speed) : 0;
  ledcWrite(M[idx].chFwd, fwdDuty);
  ledcWrite(M[idx].chRev, revDuty);
}

void motorCoast(uint8_t idx) {
  ledcWrite(M[idx].chFwd, 0);
  ledcWrite(M[idx].chRev, 0);
}

void motorBrake(uint8_t idx) {
  // "brake" keras: dua input HIGH (duty 255). Hati-hati panas kalau lama.
  ledcWrite(M[idx].chFwd, 255);
  ledcWrite(M[idx].chRev, 255);
}

void applyMotors(int m1, int m2, int m3, int m4, bool ledMaju, bool ledKanan, bool ledKiri, bool ledMundur) {
  motorWrite(0, m1);
  motorWrite(1, m2);
  motorWrite(2, m3);
  motorWrite(3, m4);
  setLED(ledMaju, ledKanan, ledKiri, ledMundur);
}

// =======================
// MOVE MODES
// =======================
enum MoveMode : uint16_t {
  MODE_STOP        = CMD_STOP,
  MODE_MAJU        = CMD_MAJU,
  MODE_MUNDUR      = CMD_MUNDUR,
  MODE_KANAN       = CMD_KANAN,
  MODE_KIRI        = CMD_KIRI,
  MODE_KANAN_ATAS  = CMD_KANAN_ATAS,
  MODE_KANAN_BAWAH = CMD_KANAN_BAWAH,
  MODE_KIRI_ATAS   = CMD_KIRI_ATAS,
  MODE_KIRI_BAWAH  = CMD_KIRI_BAWAH
};

volatile MoveMode currentMode = MODE_STOP;

int effP1() { return scaleDuty((int)p1, HREG[REG_SCALE]); }
int effP2() { return scaleDuty((int)p2, HREG[REG_SCALE]); }
int effP3() { return scaleDuty((int)p3, HREG[REG_SCALE]); }
int effP4() { return scaleDuty((int)p4, HREG[REG_SCALE]); }

void STOP_COAST_ALL() {
  currentMode = MODE_STOP;
  for (int i = 0; i < 4; i++) motorCoast(i);
  setLED(false, false, false, false);
  HREG[REG_MODE] = MODE_STOP;
}

void STOP_BRAKE_ALL() {
  currentMode = MODE_STOP;
  for (int i = 0; i < 4; i++) motorBrake(i);
  setLED(false, false, false, false);
  HREG[REG_MODE] = MODE_STOP;
}

void STOP_ALL() {
  // stop mode dari register: COAST atau BRAKE (dengan timeout brake)
  if (HREG[REG_STOPMODE] == STOP_BRAKE) {
    uint16_t ms = HREG[REG_BRAKEMS];
    if (ms < 10) ms = 10;
    if (ms > 1000) ms = 1000;

    STOP_BRAKE_ALL();
    delay(ms);            // brake sebentar
    STOP_COAST_ALL();     // lalu coast agar tidak panas
  } else {
    STOP_COAST_ALL();
  }
}

void MAJU() {
  currentMode = MODE_MAJU;
  applyMotors(+effP1(), +effP2(), +effP3(), +effP4(), true, false, false, false);
  HREG[REG_MODE] = MODE_MAJU;
}

void MUNDUR() {
  currentMode = MODE_MUNDUR;
  applyMotors(-effP1(), -effP2(), -effP3(), -effP4(), false, false, false, true);
  HREG[REG_MODE] = MODE_MUNDUR;
}

void KANAN() {
  currentMode = MODE_KANAN;
  applyMotors(-effP1(), +effP2(), -effP3(), +effP4(), false, true, false, false);
  HREG[REG_MODE] = MODE_KANAN;
}

void KIRI() {
  currentMode = MODE_KIRI;
  applyMotors(+effP1(), -effP2(), +effP3(), -effP4(), false, false, true, false);
  HREG[REG_MODE] = MODE_KIRI;
}

// diagonal (setengah roda jalan)
void KANAN_ATAS() {
  currentMode = MODE_KANAN_ATAS;
  applyMotors(0, +effP2(), 0, +effP4(), true, true, false, false);
  HREG[REG_MODE] = MODE_KANAN_ATAS;
}
void KANAN_BAWAH() {
  currentMode = MODE_KANAN_BAWAH;
  applyMotors(-effP1(), 0, -effP3(), 0, false, true, false, true);
  HREG[REG_MODE] = MODE_KANAN_BAWAH;
}
void KIRI_ATAS() {
  currentMode = MODE_KIRI_ATAS;
  applyMotors(+effP1(), 0, +effP3(), 0, true, false, true, false);
  HREG[REG_MODE] = MODE_KIRI_ATAS;
}
void KIRI_BAWAH() {
  currentMode = MODE_KIRI_BAWAH;
  applyMotors(0, -effP2(), 0, -effP4(), false, false, true, true);
  HREG[REG_MODE] = MODE_KIRI_BAWAH;
}

void reApplyMotion() {
  switch (currentMode) {
    case MODE_MAJU:        MAJU(); break;
    case MODE_MUNDUR:      MUNDUR(); break;
    case MODE_KANAN:       KANAN(); break;
    case MODE_KIRI:        KIRI(); break;
    case MODE_KANAN_ATAS:  KANAN_ATAS(); break;
    case MODE_KANAN_BAWAH: KANAN_BAWAH(); break;
    case MODE_KIRI_ATAS:   KIRI_ATAS(); break;
    case MODE_KIRI_BAWAH:  KIRI_BAWAH(); break;
    default: break; // STOP
  }
}

// =======================
// PROX update cepat (debounce ringan)
// =======================
void updateProximityRegister() {
  static uint32_t lastMs = 0;
  static uint8_t stable = 0;
  static uint8_t lastRaw = 0;

  uint32_t now = millis();
  if (now - lastMs < PROX_UPDATE_MS) return;
  lastMs = now;

  uint8_t raw = (digitalRead(PIN_PROX) == LOW) ? 1 : 0;

  if (raw == lastRaw) {
    if (stable < PROX_STABLE_COUNT) stable++;
  } else {
    stable = 0;
    lastRaw = raw;
  }

  if (stable >= PROX_STABLE_COUNT) {
    HREG[REG_PROX] = raw;
  }
}

// =======================
// Apply register changes (PWM/scale/stopmode)
// =======================
void syncConfigFromRegisters() {
  // scale
  if (HREG[REG_SCALE] > 255) HREG[REG_SCALE] = 255;

  // stopmode
  if (HREG[REG_STOPMODE] != STOP_COAST && HREG[REG_STOPMODE] != STOP_BRAKE)
    HREG[REG_STOPMODE] = STOP_COAST;

  // brake ms
  if (HREG[REG_BRAKEMS] < 10) HREG[REG_BRAKEMS] = 10;
  if (HREG[REG_BRAKEMS] > 1000) HREG[REG_BRAKEMS] = 1000;

  bool changedMotion = false;

  // PWM per motor (register 0..255)
  auto clampRegPWM = [](uint16_t &r) {
    if (r > 255) r = 255;
  };
  clampRegPWM(HREG[REG_PWM1]);
  clampRegPWM(HREG[REG_PWM2]);
  clampRegPWM(HREG[REG_PWM3]);
  clampRegPWM(HREG[REG_PWM4]);

  uint8_t rp1 = (uint8_t)HREG[REG_PWM1];
  uint8_t rp2 = (uint8_t)HREG[REG_PWM2];
  uint8_t rp3 = (uint8_t)HREG[REG_PWM3];
  uint8_t rp4 = (uint8_t)HREG[REG_PWM4];

  // deteksi perubahan config
  if (HREG[REG_SCALE] != lastScale) { lastScale = HREG[REG_SCALE]; changedMotion = true; }
  if (HREG[REG_STOPMODE] != lastStopMode) lastStopMode = HREG[REG_STOPMODE];
  if (HREG[REG_BRAKEMS] != lastBrakeMs)   lastBrakeMs = HREG[REG_BRAKEMS];

  if (rp1 != p1 || rp2 != p2 || rp3 != p3 || rp4 != p4) {
    p1 = rp1; p2 = rp2; p3 = rp3; p4 = rp4;
    // tandai NVS dirty (save nanti)
    nvsDirty = true;
    nvsDirtySinceMs = millis();
    changedMotion = true;
  }

  if (changedMotion) reApplyMotion();
}

void maybeCommitNVS() {
  if (!nvsDirty) return;
  if (millis() - nvsDirtySinceMs < NVS_SAVE_DELAY_MS) return;
  commitPWMtoNVS();
  nvsDirty = false;
}

// =======================
// Serial PWM set (opsional, seperti program i2c)
// Format: [1]200
// =======================
String pwmBuffer = "";
bool pwmMode = false;
uint32_t lastPwmCharMs = 0;

void handlePWMCommand(String cmd) {
  cmd.trim();
  if (cmd.length() < 4) return;
  if (cmd[0] != '[') return;

  int closeIdx = cmd.indexOf(']');
  if (closeIdx < 0) return;

  int motor = cmd.substring(1, closeIdx).toInt();
  int pwm   = cmd.substring(closeIdx + 1).toInt();
  if (motor < 1 || motor > 4) return;
  if (pwm < 0 || pwm > 255) return;

  // update register -> otomatis sync + simpan
  HREG[REG_PWM1 + (motor - 1)] = (uint16_t)pwm;
  syncConfigFromRegisters();
}

// =======================
// SETUP + LOOP
// =======================
void setup() {
  Serial.begin(115200);
  delay(50);

  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  // IO
  pinMode(PIN_PROX, INPUT_PULLUP);

  pinMode(L_MAJU, OUTPUT);
  pinMode(L_KANAN, OUTPUT);
  pinMode(L_MUNDUR, OUTPUT);
  pinMode(L_KIRI, OUTPUT);
  setLED(false, false, false, false);

  // LEDC init
  for (int ch = 0; ch < 8; ch++) ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(M[0].pinFwd, M[0].chFwd); ledcAttachPin(M[0].pinRev, M[0].chRev);
  ledcAttachPin(M[1].pinFwd, M[1].chFwd); ledcAttachPin(M[1].pinRev, M[1].chRev);
  ledcAttachPin(M[2].pinFwd, M[2].chFwd); ledcAttachPin(M[2].pinRev, M[2].chRev);
  ledcAttachPin(M[3].pinFwd, M[3].chFwd); ledcAttachPin(M[3].pinRev, M[3].chRev);

  // load NVS PWM
  loadPWMfromNVS();

  // init registers
  HREG[REG_CMD]      = CMD_STOP;
  HREG[REG_PROX]     = 0;
  HREG[REG_MODE]     = MODE_STOP;
  HREG[REG_SCALE]    = 255;
  HREG[REG_PWM1]     = p1;
  HREG[REG_PWM2]     = p2;
  HREG[REG_PWM3]     = p3;
  HREG[REG_PWM4]     = p4;
  HREG[REG_STOPMODE] = STOP_COAST;
  HREG[REG_BRAKEMS]  = 120;

  // modbus init
  modbus.configureHoldingRegisters(HREG, NUM_HREG);
  modbus.begin(SLAVE_ID, 9600);

  STOP_COAST_ALL();

  Serial.printf("SLAVE_ID=%d | PWM: %d %d %d %d | SCALE=%d\n",
                SLAVE_ID, p1, p2, p3, p4, HREG[REG_SCALE]);
}

void loop() {
  modbus.poll();
  updateProximityRegister();

  // ambil config terbaru dari register (kalau master ubah PWM/scale/stopmode)
  syncConfigFromRegisters();
  maybeCommitNVS();

  // eksekusi command jika berubah
  uint16_t cmd = HREG[REG_CMD];
  if (cmd != lastCmd) {
    lastCmd = cmd;
    switch (cmd) {
      case CMD_STOP:        STOP_ALL();     break;
      case CMD_MAJU:        MAJU();         break;
      case CMD_MUNDUR:      MUNDUR();       break;
      case CMD_KANAN:       KANAN();        break;
      case CMD_KIRI:        KIRI();         break;
      case CMD_KANAN_ATAS:  KANAN_ATAS();   break;
      case CMD_KANAN_BAWAH: KANAN_BAWAH();  break;
      case CMD_KIRI_ATAS:   KIRI_ATAS();    break;
      case CMD_KIRI_BAWAH:  KIRI_BAWAH();   break;
      default:              STOP_ALL();     break;
    }
  }

  // Optional: kontrol via Serial (debug)
  while (Serial.available()) {
    char c = Serial.read();
    uint32_t now = millis();

    if (pwmMode) {
      if (c == '\n' || c == '\r') {
        handlePWMCommand(pwmBuffer);
        pwmBuffer = "";
        pwmMode = false;
        continue;
      }
      pwmBuffer += c;
      lastPwmCharMs = now;
      continue;
    }

    if (c == '[') {
      pwmBuffer = "[";
      pwmMode = true;
      lastPwmCharMs = now;
      continue;
    }

    // fast debug cmd (opsional)
    switch (c) {
      case '1': HREG[REG_CMD] = CMD_MAJU; break;
      case '2': HREG[REG_CMD] = CMD_MUNDUR; break;
      case '3': HREG[REG_CMD] = CMD_KANAN; break;
      case '4': HREG[REG_CMD] = CMD_KIRI; break;
      case '5': HREG[REG_CMD] = CMD_KANAN_ATAS; break;
      case '6': HREG[REG_CMD] = CMD_KANAN_BAWAH; break;
      case '7': HREG[REG_CMD] = CMD_KIRI_ATAS; break;
      case '8': HREG[REG_CMD] = CMD_KIRI_BAWAH; break;
      case '0': HREG[REG_CMD] = CMD_STOP; break;
    }
  }

  // auto-finish PWM command kalau timeout
  if (pwmMode && millis() - lastPwmCharMs > 30) {
    handlePWMCommand(pwmBuffer);
    pwmBuffer = "";
    pwmMode = false;
  }

  delay(2);
}
