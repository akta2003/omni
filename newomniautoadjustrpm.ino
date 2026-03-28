// Untuk Kalibrasi Cukup Write ke Register 11 dengan Nilai RPM Tujuan misalkan 150
// Untuk Menggerakkan Beri Command 0-17 ke register 0 (write)
// untuk membaca prox read register 1
// untuk read pwm read register 4,5,6,7
// untuk read rpm read register 12,13,14,15


#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <Preferences.h>
#include <Wire.h>

// =======================
// ID (RS485 + I2C)
// =======================
#define SLAVE_ID   4   // <-- set 1..9
#if (SLAVE_ID >= 1) && (SLAVE_ID <= 9)
  #define I2C_ADDR  (SLAVE_ID)
#else
  #error "SLAVE_ID harus 1..9"
#endif

// =======================
// PIN DEFINITIONS
// =======================
#define RS485_DE   4
#define RS485_RX   16
#define RS485_TX   17

#define I2C_SDA   21
#define I2C_SCL   22
#define I2C_FREQ  100000

static constexpr uint8_t  PIN_PROX = 23;

// MATA (Encoder 1-to-1 Lurus)
const int encoderPin1 = 36;
const int encoderPin2 = 35;
const int encoderPin3 = 34;
const int encoderPin4 = 39;

// OTOT (Fisik Lurus)
const uint8_t PIN_M1_A1 = 27; const uint8_t PIN_M1_A2 = 14;
const uint8_t PIN_M2_A3 = 12; const uint8_t PIN_M2_A4 = 13;
const uint8_t PIN_M3_A1 = 26; const uint8_t PIN_M3_A2 = 25;
const uint8_t PIN_M4_A3 = 33; const uint8_t PIN_M4_A4 = 32;

struct MotorHW { uint8_t pinFwd; uint8_t pinRev; };
MotorHW M[4] = {
  {PIN_M2_A3, PIN_M2_A4}, // Motor 1
  {PIN_M1_A2, PIN_M1_A1}, // Motor 2
  {PIN_M4_A3, PIN_M4_A4}, // Motor 3
  {PIN_M3_A2, PIN_M3_A1}  // Motor 4
};
int8_t POL[4] = {+1, +1, +1, +1};

// LEDs
#define LED_ON  LOW
#define LED_OFF HIGH
const uint8_t L_MAJU   = 19;
const uint8_t L_KANAN  = 18;
const uint8_t L_MUNDUR = 5;
const uint8_t L_KIRI   = 15;

// =======================
// GLOBAL VARIABLES & RTOS
// =======================
ModbusRTUSlave modbus(Serial2, RS485_DE);
Preferences prefs;
TaskHandle_t TaskCore0;

enum State { STANDBY, CALIBRATING, RUNNING };
volatile State systemState = STANDBY;

// Variabel Kalibrasi
int targetRPM_Master = 150;     
int activeTargetRPM[5] = {0, 150, 150, 150, 150}; 
int calibMotor = 1;     
int calibStep = 0;      
unsigned long calibTimer = 0;
unsigned long lastPrintTimer = 0; 
int testPWM = 255;
int toleransiMasuk = 2; 
int toleransiUji = 5;   

// Variabel Operasi & Transisi (Kickstart)
volatile int targetDir[5] = {0, 0, 0, 0, 0}; 
int basePWM[5]   = {0, 0, 0, 0, 0};          
int currentPWM[5]= {0, 0, 0, 0, 0};          
volatile int memoriRPM[5] = {0, 0, 0, 0, 0}; 

// TIMER TRANSISI 
int masaTenggang = 300; // Milidetik (Jeda "Buta" sensor saat ganti arah)
volatile unsigned long blindUntilMs = 0;

bool nvsDirty = false;
unsigned long nvsDirtyTimer = 0;

// =======================
// PETA 16 HOLDING REGISTERS 
// =======================
static constexpr uint8_t NUM_HREG = 16;
uint16_t HREG[NUM_HREG];

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
static constexpr uint16_t REG_PULSEMS  = 10;
static constexpr uint16_t REG_CALIB    = 11; // Pemicu Kalibrasi
static constexpr uint16_t REG_RPM1     = 12; // Telemetri Motor 1
static constexpr uint16_t REG_RPM2     = 13; // Telemetri Motor 2
static constexpr uint16_t REG_RPM3     = 14; // Telemetri Motor 3
static constexpr uint16_t REG_RPM4     = 15; // Telemetri Motor 4

// --- DAFTAR COMMAND LENGKAP ---
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

// Command Individual Motor (Sudah Diurutkan 10 - 17)
static constexpr uint16_t CMD_M1_FWD      = 10;
static constexpr uint16_t CMD_M1_REV      = 11;
static constexpr uint16_t CMD_M2_FWD      = 12;
static constexpr uint16_t CMD_M2_REV      = 13;
static constexpr uint16_t CMD_M3_FWD      = 14;
static constexpr uint16_t CMD_M3_REV      = 15;
static constexpr uint16_t CMD_M4_FWD      = 16;
static constexpr uint16_t CMD_M4_REV      = 17;

uint16_t lastCmd = 0xFFFF;
volatile bool i2cCmdPending = false;
volatile uint16_t i2cCmdValue = 0;

// ==========================================
// FUNGSI OVERRIDE MASTER (Lurus 1-to-1)
// ==========================================
void checkManualPWM_From_Master() {
  if (HREG[REG_PWM1] != basePWM[1] || HREG[REG_PWM2] != basePWM[2] ||
      HREG[REG_PWM3] != basePWM[3] || HREG[REG_PWM4] != basePWM[4]) {
      
      basePWM[1] = constrain(HREG[REG_PWM1], 0, 255);
      basePWM[2] = constrain(HREG[REG_PWM2], 0, 255);
      basePWM[3] = constrain(HREG[REG_PWM3], 0, 255);
      basePWM[4] = constrain(HREG[REG_PWM4], 0, 255);
      
      for(int i=1; i<=4; i++) currentPWM[i] = basePWM[i]; 
      
      nvsDirty = true;
      nvsDirtyTimer = millis();
  }
}

// ==========================================
// FUNGSI I2C & PROXIMITY
// ==========================================
void onI2CReceive(int len) {
  if (len < 1) return;
  i2cCmdValue = (uint16_t)Wire.read();
  i2cCmdPending = true;
}
void onI2CRequest() { Wire.write((uint8_t)(HREG[REG_PROX] ? 1 : 0)); }

void updateProximityRegister() {
  static uint32_t lastMs = 0; static uint8_t stable = 0; static uint8_t lastRaw = 0;
  if (millis() - lastMs < 2) return;
  lastMs = millis();
  uint8_t raw = (digitalRead(PIN_PROX) == LOW) ? 1 : 0;
  if (raw == lastRaw) { if (stable < 1) stable++; } else { stable = 0; lastRaw = raw; }
  if (stable >= 1) HREG[REG_PROX] = raw;
}

// ==========================================
// FUNGSI HARDWARE MOTOR
// ==========================================
uint8_t clamp255(int v) { return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

void motorWrite(uint8_t idx, int speed) {
  int i = idx - 1; 
  speed *= POL[i];
  if (speed > 0) {
    analogWrite(M[i].pinFwd, clamp255(speed)); analogWrite(M[i].pinRev, 0); 
  } else if (speed < 0) {
    analogWrite(M[i].pinFwd, 0); analogWrite(M[i].pinRev, clamp255(-speed));
  } else {
    analogWrite(M[i].pinFwd, 0); analogWrite(M[i].pinRev, 0);
  }
}

void setLED(bool maju, bool kanan, bool kiri, bool mundur) {
  digitalWrite(L_MAJU, maju ? LED_ON : LED_OFF); digitalWrite(L_KANAN, kanan ? LED_ON : LED_OFF);
  digitalWrite(L_KIRI, kiri ? LED_ON : LED_OFF); digitalWrite(L_MUNDUR, mundur ? LED_ON : LED_OFF);
}

void stopAllMotorsBase() {
  for(int i=1; i<=4; i++) { motorWrite(i, 0); memoriRPM[i] = 0; }
  setLED(false, false, false, false);
}

void setArah(int d1, int d2, int d3, int d4, bool lMaju, bool lKanan, bool lKiri, bool lMundur) {
  targetDir[1] = d1; targetDir[2] = d2; targetDir[3] = d3; targetDir[4] = d4;
  setLED(lMaju, lKanan, lKiri, lMundur);

  // KICKSTART & MASA TENGGANG
  for(int i=1; i<=4; i++) { currentPWM[i] = basePWM[i]; }
  blindUntilMs = millis() + masaTenggang; 
}

// ==========================================
// TRANSLATOR KINEMATIKA (MENYESUAIKAN SETIR MASTER)
// ==========================================
void executeCommand(uint16_t cmd) {
  if (systemState == CALIBRATING) return; 

  switch (cmd) {
    case CMD_STOP_COAST:  
    case CMD_STOP_BRAKE:  setArah(0, 0, 0, 0, false, false, false, false); systemState = STANDBY; break;
    
    // OMNI DIRECTIONAL 
    case CMD_MAJU:        setArah(1, 1, 1, 1, true, false, false, false); systemState = RUNNING; break;
    case CMD_MUNDUR:      setArah(-1, -1, -1, -1, false, false, false, true); systemState = RUNNING; break;
    case CMD_KANAN:       setArah(1, -1, 1, -1, false, true, false, false); systemState = RUNNING; break; 
    case CMD_KIRI:        setArah(-1, 1, -1, 1, false, false, true, false); systemState = RUNNING; break; 
    case CMD_KANAN_ATAS:  setArah(1, 0, 1, 0, true, true, false, false); systemState = RUNNING; break; 
    case CMD_KANAN_BAWAH: setArah(0, -1, 0, -1, false, true, false, true); systemState = RUNNING; break; 
    case CMD_KIRI_ATAS:   setArah(0, 1, 0, 1, true, false, true, false); systemState = RUNNING; break; 
    case CMD_KIRI_BAWAH:  setArah(-1, 0, -1, 0, false, false, true, true); systemState = RUNNING; break; 
    
    // INDIVIDUAL MOTOR (Diurutkan 10-17 Sesuai Fisik Lurus)
    case CMD_M1_FWD: /* 10 */ setArah(1, 0, 0, 0, true, false, false, false); systemState = RUNNING; break;
    case CMD_M1_REV: /* 11 */ setArah(-1, 0, 0, 0, false, false, false, true); systemState = RUNNING; break;
    case CMD_M2_FWD: /* 12 */ setArah(0, 1, 0, 0, true, false, false, false); systemState = RUNNING; break;
    case CMD_M2_REV: /* 13 */ setArah(0, -1, 0, 0, false, false, false, true); systemState = RUNNING; break;
    case CMD_M3_FWD: /* 14 */ setArah(0, 0, 1, 0, true, false, false, false); systemState = RUNNING; break;
    case CMD_M3_REV: /* 15 */ setArah(0, 0, -1, 0, false, false, false, true); systemState = RUNNING; break;
    case CMD_M4_FWD: /* 16 */ setArah(0, 0, 0, 1, true, false, false, false); systemState = RUNNING; break;
    case CMD_M4_REV: /* 17 */ setArah(0, 0, 0, -1, false, false, false, true); systemState = RUNNING; break;

    default: setArah(0, 0, 0, 0, false, false, false, false); systemState = STANDBY; break;
  }
  HREG[REG_MODE] = cmd;
}

// ==========================================
// CORE 0 TASK (SANG MANAJER KOMUNIKASI)
// ==========================================
void core0Task(void * parameter) {
  for(;;) {
    modbus.poll();
    updateProximityRegister();

    // UPDATE TELEMETRI REAL-TIME UNTUK MASTER
    HREG[REG_RPM1] = memoriRPM[1];
    HREG[REG_RPM2] = memoriRPM[2];
    HREG[REG_RPM3] = memoriRPM[3];
    HREG[REG_RPM4] = memoriRPM[4];

    // LIVE UPDATE PWM KE MASTER SAAT KALIBRASI
    if (systemState == CALIBRATING) {
      HREG[REG_PWM1] = currentPWM[1];
      HREG[REG_PWM2] = currentPWM[2];
      HREG[REG_PWM3] = currentPWM[3];
      HREG[REG_PWM4] = currentPWM[4];
    } else {
      // Jika tidak kalibrasi, cek apakah ada input manual dari Master
      checkManualPWM_From_Master();
    }

    if (i2cCmdPending) {
      noInterrupts(); uint16_t c = i2cCmdValue; i2cCmdPending = false; interrupts();
      HREG[REG_CMD] = c;
    }

    if (nvsDirty && millis() - nvsDirtyTimer > 1000) {
      prefs.begin("pwm", false);
      prefs.putUChar("pwm1", basePWM[1]); prefs.putUChar("pwm2", basePWM[2]);
      prefs.putUChar("pwm3", basePWM[3]); prefs.putUChar("pwm4", basePWM[4]);
      prefs.end();
      nvsDirty = false;
    }

    if (HREG[REG_CMD] != lastCmd) {
      lastCmd = HREG[REG_CMD];
      executeCommand(lastCmd);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// CORE 1 (SANG MANDOR LAPANGAN)
// ==========================================
int getRPM(int pinEncoder) {
  unsigned long pulsa = pulseIn(pinEncoder, LOW, 1000000); 
  if (pulsa == 0) return 0; 
  int rpm = 30000000.0 / pulsa;
  return rpm;
}

void jalankanKalibrasi() {
  int pinSekarang = (calibMotor == 1) ? encoderPin1 : (calibMotor == 2) ? encoderPin2 : (calibMotor == 3) ? encoderPin3 : encoderPin4;

  if (calibStep == 0) {
    // Bersihkan layar telemetri Master dari angka sebelumnya
    for(int i=1; i<=4; i++) memoriRPM[i] = 0; 
    
    testPWM = 255; 
    currentPWM[calibMotor] = testPWM; // Kirim ke Live Telemetri PWM
    motorWrite(calibMotor, testPWM);
    
    Serial.printf("\n-> [M%d] Cek Top Speed. Tunggu 3 detik...\n", calibMotor);
    calibTimer = millis(); calibStep = 1;
  }
  else if (calibStep == 1) {
    if (millis() - calibTimer >= 3000) calibStep = 2; 
  }
  else if (calibStep == 2) {
    int maxRPM = getRPM(pinSekarang);
    if (maxRPM == 0 || maxRPM > 1000) return; 
    
    memoriRPM[calibMotor] = maxRPM; // Kirim ke Live Telemetri RPM

    activeTargetRPM[calibMotor] = targetRPM_Master; 
    if (targetRPM_Master > maxRPM) {
      activeTargetRPM[calibMotor] = maxRPM - 5; 
      if (activeTargetRPM[calibMotor] < 20) activeTargetRPM[calibMotor] = 20; 
      Serial.printf("   [M%d] PERINGATAN! Target %d di-Cap ke %d RPM.\n", calibMotor, targetRPM_Master, activeTargetRPM[calibMotor]);
    }

    float rasio = (float)activeTargetRPM[calibMotor] / (float)maxRPM;
    testPWM = constrain(rasio * 255.0, 30, 255);
    currentPWM[calibMotor] = testPWM; // Live Telemetri PWM
    motorWrite(calibMotor, testPWM);

    Serial.printf("   [M%d] Max: %d RPM. Lompat Prediksi ke PWM %d. Tunggu 2 detik...\n", calibMotor, maxRPM, testPWM);
    calibTimer = millis(); calibStep = 3; 
  }
  else if (calibStep == 3) {
    if (millis() - calibTimer >= 2000) calibStep = 4;
  }
  else if (calibStep == 4) {
    int aktualRPM = getRPM(pinSekarang);
    if (aktualRPM == 0 || aktualRPM > 1000) return; 
    
    memoriRPM[calibMotor] = aktualRPM; // Live Telemetri RPM

    int error = activeTargetRPM[calibMotor] - aktualRPM;
    int selisih = abs(error);

    if (selisih <= toleransiMasuk) {
      Serial.printf("   [M%d] Pas! (%d RPM). Uji Stabil 10 Detik...\n", calibMotor, aktualRPM);
      calibTimer = millis(); lastPrintTimer = millis(); calibStep = 5;
    } else {
      int loncatan = (selisih >= 50) ? 20 : (selisih >= 20) ? 10 : (selisih > 5) ? 4 : 2; 
      testPWM = constrain(testPWM + (error > 0 ? loncatan : -loncatan), 30, 255);
      currentPWM[calibMotor] = testPWM; // Live Telemetri PWM
      motorWrite(calibMotor, testPWM);
      
      Serial.printf("   [M%d] Aktual: %d RPM. Koreksi -> PWM %d...\n", calibMotor, aktualRPM, testPWM);
      calibTimer = millis(); calibStep = 3; 
    }
  }
  else if (calibStep == 5) {
    int aktualRPM = getRPM(pinSekarang);
    if (aktualRPM == 0 || aktualRPM > 1000) return; 
    
    memoriRPM[calibMotor] = aktualRPM; // Live Telemetri RPM
    
    unsigned long waktuBertahan = millis() - calibTimer;

    if (millis() - lastPrintTimer >= 1000) {
      Serial.printf("      Tes M%d -> %d RPM... (%d / 10 dtk)\n", calibMotor, aktualRPM, waktuBertahan/1000);
      lastPrintTimer = millis();
    }

    if (abs(activeTargetRPM[calibMotor] - aktualRPM) > toleransiUji) {
      Serial.printf("   [M%d] GAGAL UJI! Goyah di %d RPM.\n", calibMotor, aktualRPM);
      calibStep = 4; 
    } else if (waktuBertahan >= 10000) { 
      // MOTOR LULUS! Simpan dan Matikan Sejenak
      basePWM[calibMotor] = testPWM;
      motorWrite(calibMotor, 0); 
      
      calibMotor++; calibStep = 0; delay(1500); 

      // JIKA SEMUA 4 MOTOR SUDAH LULUS
      if (calibMotor > 4) {
        Serial.println("\n=== KALIBRASI VIA MASTER SELESAI ===");
        
        // Simpan Permanen ke NVS
        prefs.begin("pwm", false);
        prefs.putUChar("pwm1", basePWM[1]); prefs.putUChar("pwm2", basePWM[2]);
        prefs.putUChar("pwm3", basePWM[3]); prefs.putUChar("pwm4", basePWM[4]);
        prefs.end();

        // Pastikan Register Final Akurat
        HREG[REG_PWM1] = basePWM[1]; HREG[REG_PWM2] = basePWM[2];
        HREG[REG_PWM3] = basePWM[3]; HREG[REG_PWM4] = basePWM[4];

        HREG[REG_CALIB] = 0; // Tanda proses selesai ke Master
        
        HREG[REG_CMD] = CMD_STOP_COAST;
        lastCmd = CMD_STOP_COAST;
        setArah(0,0,0,0, false,false,false,false);

        systemState = STANDBY;
      }
    }
  }
}

void jalankanOperasi() {
  for (int i = 1; i <= 4; i++) {
    if (targetDir[i] == 0) {
      motorWrite(i, 0);
      memoriRPM[i] = 0; 
      continue; 
    }

    if (millis() < blindUntilMs) {
      motorWrite(i, currentPWM[i] * targetDir[i]); 
      continue;
    }

    int pinSekarang = (i == 1) ? encoderPin1 : (i == 2) ? encoderPin2 : (i == 3) ? encoderPin3 : encoderPin4;
    int aktualRPM = getRPM(pinSekarang); 
    
    if (aktualRPM != 0 && aktualRPM <= 1000) {
      memoriRPM[i] = aktualRPM; 
      int error = activeTargetRPM[i] - aktualRPM;
      
      if (error > 2) currentPWM[i]++;
      else if (error < -2) currentPWM[i]--;
      currentPWM[i] = constrain(currentPWM[i], 0, 255);
    }
    
    motorWrite(i, currentPWM[i] * targetDir[i]);
    delay(5); 
  }
}

// ==========================================
// SETUP MAIN 
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(50);

  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT); digitalWrite(RS485_DE, LOW);

  modbus.configureHoldingRegisters(HREG, NUM_HREG);
  modbus.begin(SLAVE_ID, 9600);
  Wire.begin(I2C_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  pinMode(PIN_PROX, INPUT_PULLUP);
  pinMode(encoderPin1, INPUT_PULLUP); pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP); pinMode(encoderPin4, INPUT_PULLUP);
  pinMode(L_MAJU, OUTPUT); pinMode(L_KANAN, OUTPUT);
  pinMode(L_MUNDUR, OUTPUT); pinMode(L_KIRI, OUTPUT);
  
  for (int i = 0; i < 4; i++) { pinMode(M[i].pinFwd, OUTPUT); pinMode(M[i].pinRev, OUTPUT); }
  
  stopAllMotorsBase();

  prefs.begin("pwm", true);
  basePWM[1] = prefs.getUChar("pwm1", 200); currentPWM[1] = basePWM[1];
  basePWM[2] = prefs.getUChar("pwm2", 200); currentPWM[2] = basePWM[2];
  basePWM[3] = prefs.getUChar("pwm3", 200); currentPWM[3] = basePWM[3];
  basePWM[4] = prefs.getUChar("pwm4", 200); currentPWM[4] = basePWM[4];
  prefs.end();

  HREG[REG_PWM1] = basePWM[1]; HREG[REG_PWM2] = basePWM[2];
  HREG[REG_PWM3] = basePWM[3]; HREG[REG_PWM4] = basePWM[4];
  HREG[REG_CMD] = CMD_STOP_COAST;
  HREG[REG_CALIB] = 0; 
  for(int i=12; i<=15; i++) HREG[i] = 0; 

  Serial.println("\n=== OMNI ROBOT FINAL: LIVE TELEMETRI ===");
  Serial.println("Kirim angka > 0 ke Register 11 untuk Auto-Kalibrasi!");

  xTaskCreatePinnedToCore(core0Task, "TaskComms", 10000, NULL, 1, &TaskCore0, 0);
}

// ==========================================
// MAIN LOOP 
// ==========================================
void loop() {
  if (HREG[REG_CALIB] > 0 && systemState != CALIBRATING) {
      targetRPM_Master = HREG[REG_CALIB];
      systemState = CALIBRATING; 
      calibMotor = 1; 
      calibStep = 0;   
      Serial.printf("\n[REMOTE TRIGGER] Master meminta kalibrasi: %d RPM\n", targetRPM_Master);
  }

  if (systemState == CALIBRATING) { jalankanKalibrasi(); } 
  else if (systemState == RUNNING) { jalankanOperasi(); } 
  else { stopAllMotorsBase(); }
}
