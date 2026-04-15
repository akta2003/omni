/**
  Write ke register 7 untuk set mode
  1 : belt jalan, proxy deteksi stop 3 detik jalan lagi
  2 : belt jalan terus, prooxy deteksi relay nyala
  3 : belt diatur oleh master, jadi hanya baca proximity dan nyalakan lampu. gerak belt diatur master
 * ============================================================
 * ESP32 UNIVERSAL CONVEYOR SLAVE - STABLE CORE STRUCTURE
 * + INTEGRASI RFID RDM6300
 * + EDGE DETECTION PROXIMITY
 * ============================================================
 */

#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <HardwareSerial.h>
#include <Wire.h>

// --- KONFIGURASI ID & PIN ---
#define SLAVE_ID         10
#define RS485_DE          4
#define RS485_RX         13
#define RS485_TX         14

#define I2C_SDA          21
#define I2C_SCL          22

// --- TAMBAHAN PIN RFID ---
#define RFID_RX          16 // UART1 RX untuk RFID RDM6300

#define PROX_PIN         34
#define RELAY_PIN        27 // DIPERBAIKI: Menghindari bentrok dengan I2C SDA

// MOTOR SINKRON (A & B)
#define MOTOR_IN1        18
#define MOTOR_IN2        19
#define MOTOR_ENA         5
#define MOTOR_IN3        17
#define MOTOR_IN4        15
#define MOTOR_ENB        23

// --- REGISTER MAP ---
static constexpr uint8_t NUM_HREG = 10;
uint16_t HREG[NUM_HREG];

static constexpr uint16_t REG_STATUS    = 0; // RFID Status (1=Ada Tag Baru)
static constexpr uint16_t REG_UID_HI    = 1;
static constexpr uint16_t REG_UID_LO    = 2;
static constexpr uint16_t REG_CHECKSUM  = 3;
static constexpr uint16_t REG_CODE      = 4; // Kode hasil mapping RFID
static constexpr uint16_t REG_MOTOR_CMD = 5; // 0:Stop, 1:Maju, 2:Mundur
static constexpr uint16_t REG_PROX_STAT = 6; // 1:Detect
static constexpr uint16_t REG_MODE_SEL  = 7; // 1:Auto, 2:Sync, 3:Master
static constexpr uint16_t REG_RELAY_CMD = 8;

// --- GLOBAL VARIABLES ---
ModbusRTUSlave modbus(Serial2, RS485_DE); // Modbus di Serial2
HardwareSerial rfidSerial(1);             // RFID di Serial1
TaskHandle_t TaskCore0;

unsigned long timerMode1 = 0;
bool mode1Active = false;

// Timer reset status RFID
unsigned long timerRFID = 0;
bool rfidActive = false;

// --- FUNGSI MOTOR ---
void applyMotor(uint16_t cmd) {
  if (cmd == 1) { // MAJU
    digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
    digitalWrite(MOTOR_ENA, HIGH); digitalWrite(MOTOR_ENB, HIGH);
  } else if (cmd == 2) { // MUNDUR
    digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
    digitalWrite(MOTOR_ENA, HIGH); digitalWrite(MOTOR_ENB, HIGH);
  } else { // STOP
    digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, LOW);
    digitalWrite(MOTOR_ENA, LOW);  digitalWrite(MOTOR_ENB, LOW);
  }
}

// --- I2C SLAVE CALLBACK ---
void onI2CReceive(int len) {
  if (len < 1) return;
  uint8_t regAddr = Wire.read();
  if (regAddr < NUM_HREG && Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    HREG[regAddr] = val;
  }
}
void onI2CRequest() { 
  uint8_t prox = HREG[REG_PROX_STAT] ? 1 : 0;
  Wire.write(prox); 
}

// --- CORE 0: MANAJER KOMUNIKASI ---
void core0Task(void * parameter) {
  for(;;) {
    modbus.poll();
    
    // Update Proximity Register agar bisa dibaca Master
    HREG[REG_PROX_STAT] = (digitalRead(PROX_PIN) == LOW) ? 1 : 0;
    
    // Sinkronisasi Motor (Membaca hasil write dari Master)
    applyMotor(HREG[REG_MOTOR_CMD]);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ============================================================
// FUNGSI RFID PARSER RDM6300
// ============================================================
static bool isHexChar(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'F') ||
         (c >= 'a' && c <= 'f');
}

bool readRDM6300(uint32_t &uid, uint8_t &cs) {
  static bool     inFrame      = false;
  static char     buf[12];
  static uint8_t  idx          = 0;
  static uint32_t frameStartMs = 0;

  while (rfidSerial.available()) {
    uint8_t b = (uint8_t)rfidSerial.read();

    if (inFrame && (millis() - frameStartMs > 50)) {
      inFrame = false; idx = 0;
    }

    if (!inFrame) {
      if (b == 0x02) {
        inFrame = true; idx = 0; frameStartMs = millis();
      }
      continue;
    }

    if (b == 0x03) {
      inFrame = false;
      if (idx != 12) { idx = 0; return false; }
      for (uint8_t i = 0; i < 12; i++) {
        if (!isHexChar(buf[i])) { idx = 0; return false; }
      }
      char idStr[11]; memcpy(idStr, buf, 10); idStr[10] = '\0';
      char csStr[3];
      csStr[0] = buf[10]; csStr[1] = buf[11]; csStr[2] = '\0';
      uid = (uint32_t)strtoul(idStr, nullptr, 16);
      cs  = (uint8_t) strtoul(csStr, nullptr, 16);
      idx = 0;
      return true;
    }

    if (idx < 12) buf[idx++] = (char)b;
    else { inFrame = false; idx = 0; return false; }
  }
  return false;
}

uint16_t mapToCode(uint32_t uid, uint8_t cs) {
  if (uid == 0xFFFFFFFFUL && cs == 0x28) return 1; 
  if (uid == 0xFFFFFFFFUL && cs == 0xE6) return 1; 
  if (uid == 0xFFFFFFFFUL && cs == 0x41) return 2; 
  if (uid == 0xFFFFFFFFUL && cs == 0x2D) return 2; 
  if (uid == 0xFFFFFFFFUL && cs == 0x8B) return 3; 
  if (uid == 0xFFFFFFFFUL && cs == 0xD6) return 3; 
  if (uid == 0xFFFFFFFFUL && cs == 0xB6) return 4;
  if (uid == 0xFFFFFFFFUL && cs == 0xDD) return 5;
  if (uid == 0xFFFFFFFFUL && cs == 0xAA) return 6;
  if (uid == 0xFFFFFFFFUL && cs == 0xE2) return 7;
  return 0; // tidak dikenal
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  // Setup Modbus Serial2
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT); digitalWrite(RS485_DE, LOW);

  // Setup RFID Serial1
  rfidSerial.begin(9600, SERIAL_8N1, RFID_RX, -1); // RX saja

  modbus.configureHoldingRegisters(HREG, NUM_HREG);
  modbus.begin(SLAVE_ID, 9600);

  // Setup I2C
  Wire.begin(SLAVE_ID, I2C_SDA, I2C_SCL, 100000);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  // Pin I/O
  pinMode(PROX_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, HIGH);
  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT); pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT); pinMode(MOTOR_ENB, OUTPUT);

  // Default Values
  HREG[REG_MODE_SEL]  = 1; // Mode 1: Auto
  HREG[REG_MOTOR_CMD] = 1; // Default Maju

  // Create Task di Core 0 khusus untuk Modbus
  xTaskCreatePinnedToCore(core0Task, "TaskComms", 10000, NULL, 1, &TaskCore0, 0);
  
  Serial.println("Sistem Conveyor Slave Berjalan...");
}

// --- CORE 1: MANDOR LAPANGAN (LOGIKA MODE & RFID) ---
void loop() {
  static uint8_t lastProx = 0; // Memori untuk edge detection sensor proximity

  // 1. PEMBACAAN RFID (Non-blocking)
  uint32_t uid = 0;
  uint8_t cs = 0;
  
  if (readRDM6300(uid, cs)) {
    uint16_t code = mapToCode(uid, cs);
    
    // Inject data ke Register Modbus/I2C
    HREG[REG_UID_HI]   = (uid >> 16) & 0xFFFF;
    HREG[REG_UID_LO]   = uid & 0xFFFF;
    HREG[REG_CHECKSUM] = cs;
    HREG[REG_CODE]     = code;
    HREG[REG_STATUS]   = 1; // Flag Tag Terbaca
    
    rfidActive = true;
    timerRFID = millis();
    
    Serial.print("RFID Terbaca! Kode: ");
    Serial.println(code);
  }

  // Auto-reset Register Status RFID setelah 3 detik
  if (rfidActive && (millis() - timerRFID >= 3000)) {
    HREG[REG_STATUS] = 0;
    rfidActive = false;
  }

  // 2. LOGIKA MODE KERJA
  uint16_t mode = HREG[REG_MODE_SEL];
  uint8_t prox = HREG[REG_PROX_STAT];

  if (mode == 1) { // MODE 1: AUTO PROXIMITY
    digitalWrite(RELAY_PIN, HIGH); // LED OFF
    
    // Edge Detection: Pemicu STOP HANYA saat prox baru saja menjadi 1 (box datang)
    if (prox == 1 && lastProx == 0 && !mode1Active) {
      HREG[REG_MOTOR_CMD] = 0; // STOP otomatis
      timerMode1 = millis();
      mode1Active = true;
      Serial.println("Box Terdeteksi! Konveyor Stop 3 Detik...");
    }
    
    // Resume motor setelah 3 detik
    if (mode1Active && (millis() - timerMode1 >= 3000)) {
      HREG[REG_MOTOR_CMD] = 1; // MAJU otomatis
      mode1Active = false;
      Serial.println("Lanjut Maju...");
    }
  } 
  else if (mode == 2) { // MODE 2: LED SYNC
    digitalWrite(RELAY_PIN, (prox == 1) ? LOW : HIGH);
    mode1Active = false;
  } 
  else if (mode == 3) { // MODE 3: MASTER CONTROL
    digitalWrite(RELAY_PIN, (prox == 1) ? LOW : HIGH);
    // Motor murni dikendalikan Master melalui write ke Register 5
    mode1Active = false;
  }

  // Simpan nilai sensor saat ini untuk putaran loop berikutnya
  lastProx = prox;

  delay(10);
}
