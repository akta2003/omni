#include "driver/ledc.h"
#include <Arduino.h>
#include <ModbusRTUSlave.h>

// =======================
// MODBUS RS485
// =======================
#define SLAVE_ID   2        // <-- GANTI: 1/2/3 sesuai conveyor
#define RS485_DE   4        // DE/RE MAX485
#define RS485_RX   16       // RX2 ESP32
#define RS485_TX   17       // TX2 ESP32

ModbusRTUSlave modbus(Serial2, RS485_DE);

// Holding registers
static constexpr uint8_t NUM_HREG = 10;
uint16_t holdingRegisters[NUM_HREG];

// Register map
static constexpr uint16_t REG_CMD  = 0;   // command dari PC
static constexpr uint16_t REG_PROX = 9999;   // proximity untuk dibaca PC (1=aktif)

// Command codes (samakan dengan PC)
static constexpr uint16_t CMD_STOP  = 0;
static constexpr uint16_t CMD_MAJU  = 1;
static constexpr uint16_t CMD_KANAN = 7;
static constexpr uint16_t CMD_KIRI  = 8;

// =======================
// PROXIMITY (aktif-LOW)
// =======================
static constexpr uint8_t  PIN_PROX = 23;          // GPIO23
static constexpr uint16_t PROX_UPDATE_MS = 10;    // update 100Hz
static constexpr uint8_t  PROX_STABLE_COUNT = 3;  // debounce sederhana

// =======================
// PWM / MOTOR (open-loop)
// =======================
static constexpr uint32_t PWM_FREQ = 20000; // 20 kHz
static constexpr uint8_t  PWM_RES  = 8;     // 0..255

// Ubah sesuai kebutuhan mekanik conveyor kamu
static constexpr int16_t PWM_FAST = 200;

// Motor pins (dari sketch kamu)
const uint8_t PIN_M1_A1 = 27; const uint8_t PIN_M1_A2 = 14;
const uint8_t PIN_M2_A3 = 12; const uint8_t PIN_M2_A4 = 13;
const uint8_t PIN_M3_A1 = 26; const uint8_t PIN_M3_A2 = 25;
const uint8_t PIN_M4_A3 = 33; const uint8_t PIN_M4_A4 = 32;

// LED indikator (opsional)
#define LED_ON  LOW
#define LED_OFF HIGH
const uint8_t L_MAJU   = 19;
const uint8_t L_KANAN  = 18;
const uint8_t L_MUNDUR = 5;
const uint8_t L_KIRI   = 15;

// LEDC channels
enum {
  CH_M1_FWD = 0, CH_M1_REV,
  CH_M2_FWD, CH_M2_REV,
  CH_M3_FWD, CH_M3_REV,
  CH_M4_FWD, CH_M4_REV
};

struct MotorHW {
  uint8_t pinFwd; uint8_t pinRev;
  uint8_t chFwd;  uint8_t chRev;
};

MotorHW M[4] = {
  {PIN_M1_A2, PIN_M1_A1, CH_M1_FWD, CH_M1_REV},
  {PIN_M2_A3, PIN_M2_A4, CH_M2_FWD, CH_M2_REV},
  {PIN_M3_A2, PIN_M3_A1, CH_M3_FWD, CH_M3_REV},
  {PIN_M4_A3, PIN_M4_A4, CH_M4_FWD, CH_M4_REV}
};

// kalau arah motor ada yang kebalik, ubah jadi -1 untuk motor itu
int8_t POL[4] = {+1, +1, +1, +1};

static inline uint8_t clamp255(int v) {
  return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

void setLED(bool maju, bool kanan, bool kiri, bool mundur) {
  digitalWrite(L_MAJU,   maju   ? LED_ON : LED_OFF);
  digitalWrite(L_KANAN,  kanan  ? LED_ON : LED_OFF);
  digitalWrite(L_KIRI,   kiri   ? LED_ON : LED_OFF);
  digitalWrite(L_MUNDUR, mundur ? LED_ON : LED_OFF);
}

// speed: -255..255
void motorWrite(uint8_t idx, int speed) {
  speed *= POL[idx];
  const uint8_t fwdDuty = speed > 0 ? clamp255(speed) : 0;
  const uint8_t revDuty = speed < 0 ? clamp255(-speed) : 0;
  ledcWrite(M[idx].chFwd, fwdDuty);
  ledcWrite(M[idx].chRev, revDuty);
}

void motorStop(uint8_t idx) {
  ledcWrite(M[idx].chFwd, 0);
  ledcWrite(M[idx].chRev, 0);
}

void STOP_ALL() {
  for (int i = 0; i < 4; i++) motorStop(i);
  setLED(false, false, false, false);
}

// Omni pattern (contoh). Sesuaikan jika mekanik kamu beda.
void setMotors(int m1, int m2, int m3, int m4, bool ledMaju, bool ledKanan, bool ledKiri, bool ledMundur) {
  motorWrite(0, m1);
  motorWrite(1, m2);
  motorWrite(2, m3);
  motorWrite(3, m4);
  setLED(ledMaju, ledKanan, ledKiri, ledMundur);
}

void MAJU()  { setMotors(+PWM_FAST, +PWM_FAST, +PWM_FAST, +PWM_FAST, true,  false, false, false); }
void KANAN() { setMotors(-PWM_FAST, +PWM_FAST, -PWM_FAST, +PWM_FAST, false, true,  false, false); }
void KIRI()  { setMotors(+PWM_FAST, -PWM_FAST, +PWM_FAST, -PWM_FAST, false, false, true,  false); }

// =======================
// PROX UPDATE -> REG_PROX (aktif-LOW)
// REG_PROX: 1 = AKTIF (terdeteksi), 0 = tidak
// =======================
void updateProximityRegister() {
  static uint32_t lastMs = 0;
  static uint8_t stable = 0;
  static uint8_t lastRaw = 0;

  uint32_t now = millis();
  if (now - lastMs < PROX_UPDATE_MS) return;
  lastMs = now;

  // aktif-LOW: pin LOW berarti sensor mendeteksi
  uint8_t raw = (digitalRead(PIN_PROX) == LOW) ? 1 : 0;

  // debounce: harus stabil beberapa kali berturut-turut
  if (raw == lastRaw) {
    if (stable < PROX_STABLE_COUNT) stable++;
  } else {
    stable = 0;
    lastRaw = raw;
  }

  if (stable >= PROX_STABLE_COUNT) {
    holdingRegisters[REG_PROX] = raw;
  }
}

// =======================
// SETUP / LOOP
// =======================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

  // Modbus
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  modbus.configureHoldingRegisters(holdingRegisters, NUM_HREG);
  modbus.begin(SLAVE_ID, 9600);

  // Proximity: aktif-LOW -> pakai pullup
  pinMode(PIN_PROX, INPUT_PULLUP);
  holdingRegisters[REG_CMD]  = CMD_STOP;
  holdingRegisters[REG_PROX] = 0;

  // LEDs
  pinMode(L_MAJU, OUTPUT);
  pinMode(L_KANAN, OUTPUT);
  pinMode(L_MUNDUR, OUTPUT);
  pinMode(L_KIRI, OUTPUT);
  setLED(false, false, false, false);

  // PWM init
  for (int ch = 0; ch < 8; ch++) ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(M[0].pinFwd, M[0].chFwd); ledcAttachPin(M[0].pinRev, M[0].chRev);
  ledcAttachPin(M[1].pinFwd, M[1].chFwd); ledcAttachPin(M[1].pinRev, M[1].chRev);
  ledcAttachPin(M[2].pinFwd, M[2].chFwd); ledcAttachPin(M[2].pinRev, M[2].chRev);
  ledcAttachPin(M[3].pinFwd, M[3].chFwd); ledcAttachPin(M[3].pinRev, M[3].chRev);

  STOP_ALL();
}

void loop() {
  // 1) Layani request Modbus (master-driven)
  modbus.poll();

  // 2) Update proximity untuk dibaca PC
  updateProximityRegister();

  // 3) Eksekusi command dari master
  static uint16_t lastCmd = 999;
  uint16_t cmd = holdingRegisters[REG_CMD];

  if (cmd != lastCmd) {
    lastCmd = cmd;

    switch (cmd) {
      case CMD_STOP:  STOP_ALL(); break;
      case CMD_MAJU:  MAJU();     break;
      case CMD_KANAN: KANAN();    break;
      case CMD_KIRI:  KIRI();     break;
      default:        STOP_ALL(); break;
    }
  }
}
