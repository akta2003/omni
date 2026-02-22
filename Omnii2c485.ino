#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <Preferences.h>
#include <Wire.h>

// =======================
// MODBUS RS485
// =======================
#define SLAVE_ID   1        // <-- GANTI: 1/2/3 sesuai omni
#define RS485_DE   4
#define RS485_RX   16
#define RS485_TX   17

ModbusRTUSlave modbus(Serial2, RS485_DE);

// =======================
// I2C SLAVE (Raspberry Pi master)
// =======================
// Mapping otomatis alamat I2C sesuai SLAVE_ID:
// 1 -> 0x01, 2 -> 0x0B, 3 -> 0x15
#if SLAVE_ID == 1
  #define I2C_ADDR  0x01
#elif SLAVE_ID == 2
  #define I2C_ADDR  0x0B
#elif SLAVE_ID == 3
  #define I2C_ADDR  0x15
#else
  #error "SLAVE_ID harus 1/2/3"
#endif

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
static constexpr uint16_t REG_STOPMODE = 8;
static constexpr uint16_t REG_BRAKEMS  = 9;
static constexpr uint16_t REG_PULSEMS  = 10;  // durasi auto-off belok/diagonal (ms)

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

// mapping "maju"
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
volatile uint16_t i2cCmdValue   = CMD_STOP;

// Base kode dari script Raspberry Pi:
// addr 0x01: 2/4/8
// addr 0x0B: 12/14/18  (base 10)
// addr 0x15: 22/24/28  (base 20)
static inline uint8_t i2cBaseFromAddr() {
  if (I2C_ADDR == 0x01) return 0;
  if (I2C_ADDR == 0x0B) return 10;
  if (I2C_ADDR == 0x15) return 20;
  return 0;
}

void onI2CReceive(int len)
{
  if (len < 1) return;
  uint8_t rx = Wire.read();

  uint16_t cmd = CMD_STOP;

  // STOP di script Pi selalu 10 untuk semua alamat
  if (rx == 10) {
    cmd = CMD_STOP;
  } else {
    uint8_t base = i2cBaseFromAddr();
    uint8_t v = (rx >= base) ? (rx - base) : rx;  // jadi 2/4/8

    if      (v == 2) cmd = CMD_MAJU;
    else if (v == 4) cmd = CMD_KANAN;
    else if (v == 8) cmd = CMD_KIRI;
    else             cmd = CMD_STOP;
  }

  i2cCmdValue   = cmd;
  i2cCmdPending = true;
}

void onI2CRequest()
{
  // Raspberry Pi: bus.read_byte(addr) => 0/1
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

void motorWrite(uint8_t idx, int speed) {
  speed *= POL[idx];
  const uint8_t fwdDuty = speed > 0 ? clamp255(speed) : 0;
  const uint8_t revDuty = speed < 0 ? clamp255(-speed) : 0;

  // Arduino-ESP32 v3.x
  ledcWriteChannel(M[idx].chFwd, fwdDuty);
  ledcWriteChannel(M[idx].chRev, revDuty);
}

void motorCoast(uint8_t idx) {
  ledcWriteChannel(M[idx].chFwd, 0);
  ledcWriteChannel(M[idx].chRev, 0);
}

void motorBrake(uint8_t idx) {
  ledcWriteChannel(M[idx].chFwd, 255);
  ledcWriteChannel(M[idx].chRev, 255);
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

// =======================
// PULSE (AUTO-OFF) STATE
// =======================
bool pulseActive = false;
uint32_t pulseUntilMs = 0;

static inline void startPulse(uint16_t ms) {
  pulseActive = true;
  pulseUntilMs = millis() + (uint32_t)ms;
}

static inline void cancelPulse() {
  pulseActive = false;
}

// =======================
// STOP / MOVE FUNCTIONS
// =======================
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
  cancelPulse();
  if (HREG[REG_STOPMODE] == STOP_BRAKE) {
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
  currentMode = MODE_MAJU;
  applyMotors(+effP1(), +effP2(), +effP3(), +effP4(), true, false, false, false);
  HREG[REG_MODE] = MODE_MAJU;
}

void MUNDUR() {
  cancelPulse();
  currentMode = MODE_MUNDUR;
  applyMotors(-effP1(), -effP2(), -effP3(), -effP4(), false, false, false, true);
  HREG[REG_MODE] = MODE_MUNDUR;
}

void KANAN() {
  currentMode = MODE_KANAN;
  applyMotors(-effP1(), +effP2(), -effP3(), +effP4(), false, true, false, false);
  HREG[REG_MODE] = MODE_KANAN;
  startPulse(HREG[REG_PULSEMS]);
}

void KIRI() {
  currentMode = MODE_KIRI;
  applyMotors(+effP1(), -effP2(), +effP3(), -effP4(), false, false, true, false);
  HREG[REG_MODE] = MODE_KIRI;
  startPulse(HREG[REG_PULSEMS]);
}

void KANAN_ATAS() {
  currentMode = MODE_KANAN_ATAS;
  applyMotors(0, +effP2(), 0, +effP4(), true, true, false, false);
  HREG[REG_MODE] = MODE_KANAN_ATAS;
  startPulse(HREG[REG_PULSEMS]);
}
void KANAN_BAWAH() {
  currentMode = MODE_KANAN_BAWAH;
  applyMotors(-effP1(), 0, -effP3(), 0, false, true, false, true);
  HREG[REG_MODE] = MODE_KANAN_BAWAH;
  startPulse(HREG[REG_PULSEMS]);
}
void KIRI_ATAS() {
  currentMode = MODE_KIRI_ATAS;
  applyMotors(+effP1(), 0, +effP3(), 0, true, false, true, false);
  HREG[REG_MODE] = MODE_KIRI_ATAS;
  startPulse(HREG[REG_PULSEMS]);
}
void KIRI_BAWAH() {
  currentMode = MODE_KIRI_BAWAH;
  applyMotors(0, -effP2(), 0, -effP4(), false, false, true, true);
  HREG[REG_MODE] = MODE_KIRI_BAWAH;
  startPulse(HREG[REG_PULSEMS]);
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
    default: break;
  }
}

// =======================
// PROX update cepat
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
// Apply register changes
// =======================
void syncConfigFromRegisters() {
  if (HREG[REG_SCALE] > 255) HREG[REG_SCALE] = 255;

  if (HREG[REG_STOPMODE] != STOP_COAST && HREG[REG_STOPMODE] != STOP_BRAKE)
    HREG[REG_STOPMODE] = STOP_COAST;

  if (HREG[REG_BRAKEMS] < 10) HREG[REG_BRAKEMS] = 10;
  if (HREG[REG_BRAKEMS] > 1000) HREG[REG_BRAKEMS] = 1000;

  if (HREG[REG_PULSEMS] < 20) HREG[REG_PULSEMS] = 20;
  if (HREG[REG_PULSEMS] > 5000) HREG[REG_PULSEMS] = 5000;

  bool changedMotion = false;

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

  if (HREG[REG_SCALE] != lastScale) { lastScale = HREG[REG_SCALE]; changedMotion = true; }
  if (HREG[REG_STOPMODE] != lastStopMode) lastStopMode = HREG[REG_STOPMODE];
  if (HREG[REG_BRAKEMS] != lastBrakeMs)   lastBrakeMs = HREG[REG_BRAKEMS];

  if (HREG[REG_PULSEMS] != lastPulseMs) {
    lastPulseMs = HREG[REG_PULSEMS];
  }

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
// Serial PWM set (opsional)
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

  HREG[REG_PWM1 + (motor - 1)] = (uint16_t)pwm;
  syncConfigFromRegisters();
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

  // LEDC init (Arduino-ESP32 v3.x)
  (void)ledcAttachChannel(M[0].pinFwd, PWM_FREQ, PWM_RES, M[0].chFwd);
  (void)ledcAttachChannel(M[0].pinRev, PWM_FREQ, PWM_RES, M[0].chRev);

  (void)ledcAttachChannel(M[1].pinFwd, PWM_FREQ, PWM_RES, M[1].chFwd);
  (void)ledcAttachChannel(M[1].pinRev, PWM_FREQ, PWM_RES, M[1].chRev);

  (void)ledcAttachChannel(M[2].pinFwd, PWM_FREQ, PWM_RES, M[2].chFwd);
  (void)ledcAttachChannel(M[2].pinRev, PWM_FREQ, PWM_RES, M[2].chRev);

  (void)ledcAttachChannel(M[3].pinFwd, PWM_FREQ, PWM_RES, M[3].chFwd);
  (void)ledcAttachChannel(M[3].pinRev, PWM_FREQ, PWM_RES, M[3].chRev);

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
  HREG[REG_PULSEMS]  = 300;
  lastPulseMs        = HREG[REG_PULSEMS];

  // Modbus init
  modbus.configureHoldingRegisters(HREG, NUM_HREG);
  modbus.begin(SLAVE_ID, 9600);

  // I2C init (slave)
  Wire.begin(I2C_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  STOP_COAST_ALL();

  Serial.printf("SLAVE_ID=%d | I2C_ADDR=0x%02X | PWM: %d %d %d %d | SCALE=%d | PULSEMS=%d\n",
                SLAVE_ID, I2C_ADDR, p1, p2, p3, p4, HREG[REG_SCALE], HREG[REG_PULSEMS]);
}

void loop() {
  // 1) layani RS485 Modbus
  modbus.poll();

  // 2) update prox (akan mengisi HREG[REG_PROX])
  updateProximityRegister();

  // 3) apply perubahan config (scale/pwm/stopmode/pulse)
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

  // 5) AUTO-OFF pulse untuk belok/diagonal
  if (pulseActive) {
    if ((int32_t)(millis() - pulseUntilMs) >= 0) {
      pulseActive = false;
      HREG[REG_CMD] = CMD_STOP; // biar master tahu sudah stop
      STOP_ALL();
      lastCmd = CMD_STOP;
    }
  }

  // 6) eksekusi command jika berubah
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

  if (pwmMode && millis() - lastPwmCharMs > 30) {
    handlePWMCommand(pwmBuffer);
    pwmBuffer = "";
    pwmMode = false;
  }

  delay(2);
}
