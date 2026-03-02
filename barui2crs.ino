#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_arduino_version.h>

// =======================
// ID (RS485 + I2C)
// =======================
#define SLAVE_ID   4   // <-- set 1..9

#if (SLAVE_ID >= 1) && (SLAVE_ID <= 9)
  #define I2C_ADDR  (SLAVE_ID)   // 1..9 => 0x01..0x09
#else
  #error "SLAVE_ID harus 1..9"
#endif

// =======================
// MODBUS RS485
// =======================
#define RS485_DE   4
#define RS485_RX   16
#define RS485_TX   17

ModbusRTUSlave modbus(Serial2, RS485_DE);

// =======================
// I2C SLAVE (Raspberry Pi master)
// =======================
#define I2C_SDA   21
#define I2C_SCL   22
#define I2C_FREQ  100000  // 100kHz

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
static constexpr uint16_t REG_STOPMODE = 8;   // tetap ada (config)
static constexpr uint16_t REG_BRAKEMS  = 9;   // brake duration (ms)
static constexpr uint16_t REG_PULSEMS  = 10;  // auto-off belok/diagonal (ms), 0 = disable

// =======================
// COMMAND CODES
// =======================
// 8 arah + stop
static constexpr uint16_t CMD_STOP_COAST  = 0;
static constexpr uint16_t CMD_MAJU        = 1;
static constexpr uint16_t CMD_KANAN_ATAS  = 2;
static constexpr uint16_t CMD_KANAN       = 3;
static constexpr uint16_t CMD_KANAN_BAWAH = 4;
static constexpr uint16_t CMD_MUNDUR      = 5;
static constexpr uint16_t CMD_KIRI_BAWAH  = 6;
static constexpr uint16_t CMD_KIRI        = 7;
static constexpr uint16_t CMD_KIRI_ATAS   = 8;
static constexpr uint16_t CMD_STOP_BRAKE  = 9;

// command tambahan: single motor (8 command)
// (nomor kamu biarkan seperti ini)
static constexpr uint16_t CMD_M1_FWD = 12; //motor2
static constexpr uint16_t CMD_M1_REV = 13; //motor2
static constexpr uint16_t CMD_M2_FWD = 10; //motor1
static constexpr uint16_t CMD_M2_REV = 11; //motor1
static constexpr uint16_t CMD_M3_FWD = 16; //motor4
static constexpr uint16_t CMD_M3_REV = 17; //motor4
static constexpr uint16_t CMD_M4_FWD = 14; //motor3
static constexpr uint16_t CMD_M4_REV = 15; //motor3

// stop mode (config register)
static constexpr uint16_t STOP_COAST = 0;
static constexpr uint16_t STOP_BRAKE = 1;

// =======================
// PROXIMITY (aktif-LOW)
// =======================
static constexpr uint8_t  PIN_PROX = 23;
static constexpr uint16_t PROX_UPDATE_MS    = 2;
static constexpr uint8_t  PROX_STABLE_COUNT = 1;

// =======================
// PWM / MOTOR
// =======================
static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t  PWM_RES  = 8; // 0..255 untuk 8-bit

// Motor pins (MX1508 = 2 pin per motor)
const uint8_t PIN_M1_A1 = 27; const uint8_t PIN_M1_A2 = 14;
const uint8_t PIN_M2_A3 = 12; const uint8_t PIN_M2_A4 = 13;
const uint8_t PIN_M3_A1 = 26; const uint8_t PIN_M3_A2 = 25;
const uint8_t PIN_M4_A3 = 33; const uint8_t PIN_M4_A4 = 32;

// LED indikator (opsional)
#define LED_ON  LOW
#define LED_OFF HIGH
const uint8_t L_MAJU   = 19;
const uint8_t L_KANAN  = 18;
const uint8_t L_MUNDUR = 5;   // bootstrap pin
const uint8_t L_KIRI   = 15;  // bootstrap pin

// =======================
// POWER-ON LED SELF TEST
// =======================
static constexpr uint16_t LED_SELFTEST_MS = 250; // durasi semua LED nyala saat power-on (ms)

// LEDC channels (tetap disimpan untuk core 2.x)
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

// mapping "maju" (sesuai kode lama)
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
  int sign = (duty >= 0) ? 1 : -1;
  int mag  = abs(duty);
  int out  = (mag * (int)scale255) / 255;
  return sign * out;
}

// =======================
// LEDC COMPAT LAYER (core 2.x vs 3.x)
// =======================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static inline void pwmAttach(uint8_t pin, uint8_t /*ch*/, uint32_t freq, uint8_t resBits) {
    (void)ledcAttach(pin, freq, resBits); // auto channel
  }
  static inline void pwmWrite(uint8_t pin, uint8_t /*ch*/, uint32_t duty) {
    ledcWrite(pin, duty);
  }
#else
  static inline void pwmAttach(uint8_t pin, uint8_t ch, uint32_t freq, uint8_t resBits) {
    ledcSetup(ch, freq, resBits);
    ledcAttachPin(pin, ch);
  }
  static inline void pwmWrite(uint8_t /*pin*/, uint8_t ch, uint32_t duty) {
    ledcWrite(ch, duty);
  }
#endif

static inline uint32_t pwmMaxDuty(uint8_t resBits) {
  return (resBits >= 31) ? 0x7FFFFFFFul : ((1UL << resBits) - 1UL);
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
uint16_t lastPulseMs = 300;

// save debouncing
bool nvsDirty = false;
uint32_t nvsDirtySinceMs = 0;
static constexpr uint32_t NVS_SAVE_DELAY_MS = 800;

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
// I2C BRIDGE (pending command)
// =======================
volatile bool     i2cCmdPending = false;
volatile uint16_t i2cCmdValue   = CMD_STOP_COAST;

// ✅ FIX: validasi command harus menerima 10,11,16,17 juga
static inline bool isValidCmd(uint16_t c) {
  // 0..9 valid
  if (c <= CMD_STOP_BRAKE) return true;
  // 10..17 valid (single-motor)
  if (c >= 10 && c <= 17)  return true;
  return false;
}

void onI2CReceive(int len) {
  if (len < 1) return;

  // Versi kamu: kirim 1 byte.
  // Kalau master ternyata mengirim lebih dari 1 byte, kita ambil byte TERAKHIR
  // agar tidak "nyangkut" data sisa.
  uint8_t rx = 0;
  while (Wire.available()) {
    rx = Wire.read();
  }

  uint16_t cmd = (uint16_t)rx;

  if (!isValidCmd(cmd)) cmd = CMD_STOP_COAST;

  i2cCmdValue   = cmd;
  i2cCmdPending = true;
}

void onI2CRequest() {
  Wire.write((uint8_t)(HREG[REG_PROX] ? 1 : 0));
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

// nyalakan semua LED sebentar saat power-on
void ledSelfTest() {
  setLED(true, true, true, true);
  delay(LED_SELFTEST_MS);
  setLED(false, false, false, false);
  delay(50);
}

void motorWrite(uint8_t idx, int speed) {
  speed *= POL[idx];
  const uint32_t fwdDuty = speed > 0 ? clamp255(speed) : 0;
  const uint32_t revDuty = speed < 0 ? clamp255(-speed) : 0;

  pwmWrite(M[idx].pinFwd, M[idx].chFwd, fwdDuty);
  pwmWrite(M[idx].pinRev, M[idx].chRev, revDuty);
}

void motorCoast(uint8_t idx) {
  pwmWrite(M[idx].pinFwd, M[idx].chFwd, 0);
  pwmWrite(M[idx].pinRev, M[idx].chRev, 0);
}

void motorBrake(uint8_t idx) {
  const uint32_t maxD = pwmMaxDuty(PWM_RES); // untuk 8-bit = 255
  pwmWrite(M[idx].pinFwd, M[idx].chFwd, maxD);
  pwmWrite(M[idx].pinRev, M[idx].chRev, maxD);
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
volatile uint16_t currentMode = CMD_STOP_COAST;

int effP1() { return scaleDuty((int)p1, HREG[REG_SCALE]); }
int effP2() { return scaleDuty((int)p2, HREG[REG_SCALE]); }
int effP3() { return scaleDuty((int)p3, HREG[REG_SCALE]); }
int effP4() { return scaleDuty((int)p4, HREG[REG_SCALE]); }

// =======================
// PULSE (AUTO-OFF) untuk belok/diagonal saja
// =======================
bool pulseActive = false;
uint32_t pulseUntilMs = 0;

static inline void startPulse(uint16_t ms) {
  if (ms == 0) return;
  pulseActive = true;
  pulseUntilMs = millis() + (uint32_t)ms;
}
static inline void cancelPulse() { pulseActive = false; }

// =======================
// STOP / MOVE FUNCTIONS
// =======================
void STOP_COAST_ALL() {
  currentMode = CMD_STOP_COAST;
  for (int i = 0; i < 4; i++) motorCoast(i);
  setLED(false, false, false, false);
  HREG[REG_MODE] = CMD_STOP_COAST;
}

void STOP_BRAKE_ALL() {
  currentMode = CMD_STOP_COAST;
  for (int i = 0; i < 4; i++) motorBrake(i);
  setLED(false, false, false, false);
  HREG[REG_MODE] = CMD_STOP_COAST;
}

void STOP_ALL(int overrideStopMode /* -1=pakai REG_STOPMODE, 0=coast, 1=brake */) {
  cancelPulse();

  uint16_t mode = (overrideStopMode < 0) ? HREG[REG_STOPMODE] : (uint16_t)overrideStopMode;
  if (mode != STOP_COAST && mode != STOP_BRAKE) mode = STOP_COAST;

  if (mode == STOP_BRAKE) {
    uint16_t ms = HREG[REG_BRAKEMS];
    if (ms < 10) ms = 10;
    if (ms > 1000) ms = 1000;

    STOP_BRAKE_ALL();
    delay(ms);
    STOP_COAST_ALL();
  } else {
    STOP_COAST_ALL();
  }
}

void MAJU() {
  cancelPulse();
  currentMode = CMD_MAJU;
  applyMotors(+effP1(), +effP2(), +effP3(), +effP4(), true, false, false, false);
  HREG[REG_MODE] = CMD_MAJU;
}

void MUNDUR() {
  cancelPulse();
  currentMode = CMD_MUNDUR;
  applyMotors(-effP1(), -effP2(), -effP3(), -effP4(), false, false, false, true);
  HREG[REG_MODE] = CMD_MUNDUR;
}

void KANAN() {
  currentMode = CMD_KANAN;
  applyMotors(-effP1(), +effP2(), -effP3(), +effP4(), false, true, false, false);
  HREG[REG_MODE] = CMD_KANAN;
  startPulse(HREG[REG_PULSEMS]);
}

void KIRI() {
  currentMode = CMD_KIRI;
  applyMotors(+effP1(), -effP2(), +effP3(), -effP4(), false, false, true, false);
  HREG[REG_MODE] = CMD_KIRI;
  startPulse(HREG[REG_PULSEMS]);
}

void KANAN_ATAS() {
  currentMode = CMD_KANAN_ATAS;
  applyMotors(0, +effP2(), 0, +effP4(), true, true, false, false);
  HREG[REG_MODE] = CMD_KANAN_ATAS;
  startPulse(HREG[REG_PULSEMS]);
}

void KANAN_BAWAH() {
  currentMode = CMD_KANAN_BAWAH;
  applyMotors(-effP1(), 0, -effP3(), 0, false, true, false, true);
  HREG[REG_MODE] = CMD_KANAN_BAWAH;
  startPulse(HREG[REG_PULSEMS]);
}

void KIRI_ATAS() {
  currentMode = CMD_KIRI_ATAS;
  applyMotors(+effP1(), 0, +effP3(), 0, true, false, true, false);
  HREG[REG_MODE] = CMD_KIRI_ATAS;
  startPulse(HREG[REG_PULSEMS]);
}

void KIRI_BAWAH() {
  currentMode = CMD_KIRI_BAWAH;
  applyMotors(0, -effP2(), 0, -effP4(), false, false, true, true);
  HREG[REG_MODE] = CMD_KIRI_BAWAH;
  startPulse(HREG[REG_PULSEMS]);
}

// =======================
// SINGLE MOTOR COMMANDS (TIDAK PAKAI PULSE)
// =======================
void M1_FWD() {
  cancelPulse();
  currentMode = CMD_M1_FWD;
  applyMotors(+effP1(), 0, 0, 0, true, false, false, false);
  HREG[REG_MODE] = CMD_M1_FWD;
}
void M1_REV() {
  cancelPulse();
  currentMode = CMD_M1_REV;
  applyMotors(-effP1(), 0, 0, 0, false, false, false, true);
  HREG[REG_MODE] = CMD_M1_REV;
}
void M2_FWD() {
  cancelPulse();
  currentMode = CMD_M2_FWD;
  applyMotors(0, +effP2(), 0, 0, true, false, false, false);
  HREG[REG_MODE] = CMD_M2_FWD;
}
void M2_REV() {
  cancelPulse();
  currentMode = CMD_M2_REV;
  applyMotors(0, -effP2(), 0, 0, false, false, false, true);
  HREG[REG_MODE] = CMD_M2_REV;
}
void M3_FWD() {
  cancelPulse();
  currentMode = CMD_M3_FWD;
  applyMotors(0, 0, +effP3(), 0, true, false, false, false);
  HREG[REG_MODE] = CMD_M3_FWD;
}
void M3_REV() {
  cancelPulse();
  currentMode = CMD_M3_REV;
  applyMotors(0, 0, -effP3(), 0, false, false, false, true);
  HREG[REG_MODE] = CMD_M3_REV;
}
void M4_FWD() {
  cancelPulse();
  currentMode = CMD_M4_FWD;
  applyMotors(0, 0, 0, +effP4(), true, false, false, false);
  HREG[REG_MODE] = CMD_M4_FWD;
}
void M4_REV() {
  cancelPulse();
  currentMode = CMD_M4_REV;
  applyMotors(0, 0, 0, -effP4(), false, false, false, true);
  HREG[REG_MODE] = CMD_M4_REV;
}

void reApplyMotion() {
  switch (currentMode) {
    case CMD_MAJU:        MAJU();        break;
    case CMD_MUNDUR:      MUNDUR();      break;
    case CMD_KANAN:       KANAN();       break;
    case CMD_KIRI:        KIRI();        break;
    case CMD_KANAN_ATAS:  KANAN_ATAS();  break;
    case CMD_KANAN_BAWAH: KANAN_BAWAH(); break;
    case CMD_KIRI_ATAS:   KIRI_ATAS();   break;
    case CMD_KIRI_BAWAH:  KIRI_BAWAH();  break;

    case CMD_M1_FWD:      M1_FWD();      break;
    case CMD_M1_REV:      M1_REV();      break;
    case CMD_M2_FWD:      M2_FWD();      break;
    case CMD_M2_REV:      M2_REV();      break;
    case CMD_M3_FWD:      M3_FWD();      break;
    case CMD_M3_REV:      M3_REV();      break;
    case CMD_M4_FWD:      M4_FWD();      break;
    case CMD_M4_REV:      M4_REV();      break;

    default: /* STOP */   break;
  }
}

// =======================
// PROX update cepat -> HREG[REG_PROX]
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
// Apply register changes (PWM/Scale/Stop params)
// =======================
void syncConfigFromRegisters() {
  if (HREG[REG_SCALE] > 255) HREG[REG_SCALE] = 255;

  if (HREG[REG_STOPMODE] != STOP_COAST && HREG[REG_STOPMODE] != STOP_BRAKE)
    HREG[REG_STOPMODE] = STOP_COAST;

  if (HREG[REG_BRAKEMS] < 10) HREG[REG_BRAKEMS] = 10;
  if (HREG[REG_BRAKEMS] > 1000) HREG[REG_BRAKEMS] = 1000;

  if (HREG[REG_PULSEMS] > 5000) HREG[REG_PULSEMS] = 5000;

  bool changedMotion = false;

  auto clampRegPWM = [](uint16_t &r) { if (r > 255) r = 255; };
  clampRegPWM(HREG[REG_PWM1]);
  clampRegPWM(HREG[REG_PWM2]);
  clampRegPWM(HREG[REG_PWM3]);
  clampRegPWM(HREG[REG_PWM4]);

  uint8_t rp1 = (uint8_t)HREG[REG_PWM1];
  uint8_t rp2 = (uint8_t)HREG[REG_PWM2];
  uint8_t rp3 = (uint8_t)HREG[REG_PWM3];
  uint8_t rp4 = (uint8_t)HREG[REG_PWM4];

  if (HREG[REG_SCALE] != lastScale) { lastScale = HREG[REG_SCALE]; changedMotion = true; }
  if (HREG[REG_STOPMODE] != lastStopMode) lastStopMode = HREG[REG_STOPMODE];
  if (HREG[REG_BRAKEMS] != lastBrakeMs)   lastBrakeMs = HREG[REG_BRAKEMS];
  if (HREG[REG_PULSEMS] != lastPulseMs)   lastPulseMs = HREG[REG_PULSEMS];

  if (rp1 != p1 || rp2 != p2 || rp3 != p3 || rp4 != p4) {
    p1 = rp1; p2 = rp2; p3 = rp3; p4 = rp4;
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
// SETUP + LOOP
// =======================
void setup() {
  Serial.begin(115200);
  delay(50);

  // RS485 UART
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

  // --- TEST LED saat power on (semua nyala sebentar) ---
  ledSelfTest();

  // LEDC init
  pwmAttach(M[0].pinFwd, M[0].chFwd, PWM_FREQ, PWM_RES);
  pwmAttach(M[0].pinRev, M[0].chRev, PWM_FREQ, PWM_RES);

  pwmAttach(M[1].pinFwd, M[1].chFwd, PWM_FREQ, PWM_RES);
  pwmAttach(M[1].pinRev, M[1].chRev, PWM_FREQ, PWM_RES);

  pwmAttach(M[2].pinFwd, M[2].chFwd, PWM_FREQ, PWM_RES);
  pwmAttach(M[2].pinRev, M[2].chRev, PWM_FREQ, PWM_RES);

  pwmAttach(M[3].pinFwd, M[3].chFwd, PWM_FREQ, PWM_RES);
  pwmAttach(M[3].pinRev, M[3].chRev, PWM_FREQ, PWM_RES);

  for (int i = 0; i < 4; i++) motorCoast(i);

  // load NVS PWM
  loadPWMfromNVS();

  // init registers
  HREG[REG_CMD]      = CMD_STOP_COAST;
  HREG[REG_PROX]     = 0;
  HREG[REG_MODE]     = CMD_STOP_COAST;
  HREG[REG_SCALE]    = 255;
  HREG[REG_PWM1]     = p1;
  HREG[REG_PWM2]     = p2;
  HREG[REG_PWM3]     = p3;
  HREG[REG_PWM4]     = p4;
  HREG[REG_STOPMODE] = STOP_COAST;
  HREG[REG_BRAKEMS]  = 120;
  HREG[REG_PULSEMS]  = 0;
  lastPulseMs        = HREG[REG_PULSEMS];

  // Modbus init
  modbus.configureHoldingRegisters(HREG, NUM_HREG);
  modbus.begin(SLAVE_ID, 9600);

  // I2C init (slave)
  Wire.begin(I2C_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  STOP_COAST_ALL();

  Serial.printf("SLAVE_ID=%d | I2C_ADDR=0x%02X | PWM: %d %d %d %d | SCALE=%d | BRAKEMS=%d | PULSEMS=%d\n",
                SLAVE_ID, I2C_ADDR, p1, p2, p3, p4,
                HREG[REG_SCALE], HREG[REG_BRAKEMS], HREG[REG_PULSEMS]);
}

void loop() {
  // 1) layani RS485 Modbus
  modbus.poll();

  // 2) update prox -> HREG[REG_PROX]
  updateProximityRegister();

  // 3) apply perubahan config (scale/pwm/stopmode/brakems/pulse)
  syncConfigFromRegisters();
  maybeCommitNVS();

  // 4) terima command dari I2C (1 byte) -> tulis ke HREG[REG_CMD]
  if (i2cCmdPending) {
    noInterrupts();
    uint16_t cmd = i2cCmdValue;
    i2cCmdPending = false;
    interrupts();
    HREG[REG_CMD] = cmd;
  }

  // 5) AUTO-OFF pulse untuk belok/diagonal SAJA
  if (pulseActive) {
    if ((int32_t)(millis() - pulseUntilMs) >= 0) {
      pulseActive = false;
      HREG[REG_CMD] = CMD_STOP_COAST;
      STOP_ALL(STOP_COAST);
      lastCmd = CMD_STOP_COAST;
    }
  }

  // 6) eksekusi command jika berubah
  uint16_t cmd = HREG[REG_CMD];
  if (cmd != lastCmd) {
    lastCmd = cmd;

    switch (cmd) {
      case CMD_STOP_COAST:  STOP_ALL(STOP_COAST);  break;
      case CMD_STOP_BRAKE:  STOP_ALL(STOP_BRAKE);  break;

      case CMD_MAJU:        MAJU();        break;
      case CMD_KANAN_ATAS:  KANAN_ATAS();  break;
      case CMD_KANAN:       KANAN();       break;
      case CMD_KANAN_BAWAH: KANAN_BAWAH(); break;
      case CMD_MUNDUR:      MUNDUR();      break;
      case CMD_KIRI_BAWAH:  KIRI_BAWAH();  break;
      case CMD_KIRI:        KIRI();        break;
      case CMD_KIRI_ATAS:   KIRI_ATAS();   break;

      // single motor
      case CMD_M1_FWD:      M1_FWD();      break;
      case CMD_M1_REV:      M1_REV();      break;
      case CMD_M2_FWD:      M2_FWD();      break;
      case CMD_M2_REV:      M2_REV();      break;
      case CMD_M3_FWD:      M3_FWD();      break;
      case CMD_M3_REV:      M3_REV();      break;
      case CMD_M4_FWD:      M4_FWD();      break;
      case CMD_M4_REV:      M4_REV();      break;

      default:              STOP_ALL(STOP_COAST);  break;
    }
  }

  delay(2);
}
