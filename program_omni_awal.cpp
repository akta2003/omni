#include "driver/ledc.h" // sudah ada di core ESP32
#include <Arduino.h>
#include <ModbusRTUSlave.h> // Library Wajib

// ====== KONFIGURASI MODBUS RS485 ======
#define SLAVE_ID 4        // <--- GANTI JADI 1 JIKA UNTUK SLAVE PERTAMA
#define RS485_READ 4      // Pin DE/RE
#define RS485_RO   16     // Pin RX2
#define RS485_DI   17     // Pin TX2

ModbusRTUSlave modbus(Serial2, RS485_READ);
const uint8_t numHoldingRegisters = 10;
uint16_t holdingRegisters[numHoldingRegisters];

// ====== KONFIG PWM ======
static constexpr uint32_t PWM_FREQ = 20000; // 20 kHz (silent)
static constexpr uint8_t PWM_RES = 8;       // 8-bit (0..255)
#define LED_ON LOW
#define LED_OFF HIGH

// ====== ENCODER / KECEPATAN ======
static constexpr float PULSES_PER_REV = 2.0f;      // encoder 2 PPR
static constexpr uint32_t RPM_TIMEOUT_US = 500000; // 0.5 s tanpa pulsa -> dianggap 0 RPM
static constexpr float RPM_EMA_ALPHA = 0.30f;      // smoothing RPM (0..1)

// ====== LOOP KONTROL ======
static constexpr uint16_t CTRL_PERIOD_MS = 20; // 50 Hz kontrol
static constexpr float DT = CTRL_PERIOD_MS / 1000.0f;

// PI + Feedforward (satuan: pwm / rpm)
static constexpr float KFF = 1.20f; // feedforward
static constexpr float KP = 0.60f;  // proporsional
static constexpr float KI = 0.15f;  // integral

// Batas aktuator
static constexpr int16_t PWM_MAX = 255;
static constexpr int16_t PWM_MIN_RUN = 40;  
static constexpr uint8_t SLEW_PER_TICK = 8; 

// ====== KONFIG CETAK RPM ======
static constexpr uint32_t SPEED_REPORT_MS = 1000; 

// Motor pins
const uint8_t PIN_M1_A1 = 27; const uint8_t PIN_M1_A2 = 14;
const uint8_t PIN_M2_A3 = 12; const uint8_t PIN_M2_A4 = 13;
const uint8_t PIN_M3_A1 = 26; const uint8_t PIN_M3_A2 = 25;
const uint8_t PIN_M4_A3 = 33; const uint8_t PIN_M4_A4 = 32;

// Encoder pins
const uint8_t PIN_ENC1_M1 = 35;
const uint8_t PIN_ENC2_M2 = 34;
const uint8_t PIN_ENC3_M3 = 39;
const uint8_t PIN_ENC4_M4 = 36;

// LEDs
const uint8_t L_MAJU = 19;
const uint8_t L_KANAN = 18;
const uint8_t L_MUNDUR = 5; 
const uint8_t L_KIRI = 15;  

// ====== LEDC CHANNEL ASSIGN ======
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

int8_t POL[4] = {+1, +1, +1, +1};

// ====== STATE ENCODER (ISR) ======
volatile uint32_t encCount[4] = {0, 0, 0, 0};        
volatile uint32_t lastEdgeUs[4] = {0, 0, 0, 0};      
volatile uint32_t lastPeriodUsRaw[4] = {0, 0, 0, 0}; 

static inline bool edge_ok(uint8_t i, uint32_t now) {
    uint32_t dt = now - lastEdgeUs[i];
    if (dt < 1000) return false; 
    lastEdgeUs[i] = now;
    return true;
}

void IRAM_ATTR isrEnc0() { uint32_t t=micros(); if(edge_ok(0,t)){ encCount[0]++; lastPeriodUsRaw[0]=t-lastPeriodUsRaw[0]?t-(t-lastPeriodUsRaw[0]):0; } }
void IRAM_ATTR isrEnc1() { uint32_t t=micros(); if(edge_ok(1,t)){ encCount[1]++; lastPeriodUsRaw[1]=t-lastPeriodUsRaw[1]?t-(t-lastPeriodUsRaw[1]):0; } }
void IRAM_ATTR isrEnc2() { uint32_t t=micros(); if(edge_ok(2,t)){ encCount[2]++; lastPeriodUsRaw[2]=t-lastPeriodUsRaw[2]?t-(t-lastPeriodUsRaw[2]):0; } }
void IRAM_ATTR isrEnc3() { uint32_t t=micros(); if(edge_ok(3,t)){ encCount[3]++; lastPeriodUsRaw[3]=t-lastPeriodUsRaw[3]?t-(t-lastPeriodUsRaw[3]):0; } }

// ====== STATE ESTIMASI & KONTROL ======
float rpmEMA[4] = {0, 0, 0, 0};    
float targetRPM[4] = {0, 0, 0, 0}; 
float integ[4] = {0, 0, 0, 0};     
int16_t uPWM[4] = {0, 0, 0, 0};    

inline uint8_t clamp255(int v) { return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); }

void motorWrite(uint8_t idx, int speed) {
    speed *= POL[idx];
    const uint8_t fwdDuty = speed > 0 ? clamp255(speed) : 0;
    const uint8_t revDuty = speed < 0 ? clamp255(-speed) : 0;
    ledcWrite(M[idx].chFwd, fwdDuty);
    ledcWrite(M[idx].chRev, revDuty);
}

void motorCoast(uint8_t idx) { ledcWrite(M[idx].chFwd, 0); ledcWrite(M[idx].chRev, 0); }
void motorBrake(uint8_t idx) { ledcWrite(M[idx].chFwd, 255); ledcWrite(M[idx].chRev, 255); }

void setLED(bool maju, bool kanan, bool kiri, bool mundur) {
    digitalWrite(L_MAJU, maju ? LED_ON : LED_OFF);
    digitalWrite(L_KANAN, kanan ? LED_ON : LED_OFF);
    digitalWrite(L_KIRI, kiri ? LED_ON : LED_OFF);
    digitalWrite(L_MUNDUR, mundur ? LED_ON : LED_OFF);
}

void STOP_COAST() {
    for (int i = 0; i < 4; i++) { motorCoast(i); targetRPM[i] = 0; integ[i] = 0; uPWM[i] = 0; }
    setLED(false, false, false, false);
}
void STOP_BRAKE() {
    for (int i = 0; i < 4; i++) { motorBrake(i); targetRPM[i] = 0; integ[i] = 0; uPWM[i] = 0; }
    setLED(false, false, false, false);
}

// ====== ESTIMASI RPM ======
void estimateRPMs() {
    uint32_t lastEdge[4], periodRaw[4];
    noInterrupts();
    for (int i = 0; i < 4; i++) { lastEdge[i] = lastEdgeUs[i]; periodRaw[i] = lastPeriodUsRaw[i]; }
    interrupts();
    uint32_t now = micros();
    for (int i = 0; i < 4; i++) {
        float rpmInst = 0.0f;
        if (periodRaw[i] > 0 && (now - lastEdge[i]) < RPM_TIMEOUT_US) {
            rpmInst = 60000000.0f / (PULSES_PER_REV * (float)periodRaw[i]);
        }
        rpmEMA[i] = RPM_EMA_ALPHA * rpmInst + (1.0f - RPM_EMA_ALPHA) * rpmEMA[i];
    }
}

// ====== KONTROL KECEPATAN PID ======
void speedControllerTick() {
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < CTRL_PERIOD_MS) return;
    lastMs = now;
    estimateRPMs();

    for (int i = 0; i < 4; i++) {
        float sp = targetRPM[i];
        float meas = (sp >= 0 ? +rpmEMA[i] : -rpmEMA[i]);
        float err = sp - meas;
        float ff = KFF * sp;
        integ[i] += (KI * err) * DT;
        if (integ[i] > 200) integ[i] = 200; if (integ[i] < -200) integ[i] = -200;
        float u = ff + KP * err + integ[i];
        if (sp == 0.0f) { u = 0.0f; integ[i] = 0.0f; } 
        else if (fabsf(u) < PWM_MIN_RUN) u = copysignf(PWM_MIN_RUN, u);
        
        if (u > PWM_MAX) u = PWM_MAX; if (u < -PWM_MAX) u = -PWM_MAX;
        int16_t targetPWM = (int16_t)lroundf(u);
        int16_t du = targetPWM - uPWM[i];
        if (du > SLEW_PER_TICK) du = SLEW_PER_TICK; if (du < -SLEW_PER_TICK) du = -SLEW_PER_TICK;
        uPWM[i] += du;
        motorWrite(i, uPWM[i]);
    }
}

// ====== GERAK ======
void setTargetsRPM(float m1, float m2, float m3, float m4, bool ledMaju, bool ledKanan, bool ledKiri, bool ledMundur) {
    targetRPM[0] = m1; targetRPM[1] = m2; targetRPM[2] = m3; targetRPM[3] = m4;
    setLED(ledMaju, ledKanan, ledKiri, ledMundur);
}

// Preset Speed
static constexpr float RPM_SLOW = 110.0f;
static constexpr float RPM_MID = 140.0f;
static constexpr float RPM_FAST = 160.0f;

void MAJU() { setTargetsRPM(+RPM_FAST, +RPM_FAST, +RPM_FAST, +RPM_FAST, true, false, false, false); }
void MUNDUR() { setTargetsRPM(-RPM_FAST, -RPM_FAST, -RPM_FAST, -RPM_FAST, false, false, false, true); }
void KANAN() { setTargetsRPM(-RPM_FAST, +RPM_FAST, -RPM_FAST, +RPM_FAST, false, true, false, false); }
void KIRI() { setTargetsRPM(+RPM_FAST, -RPM_FAST, +RPM_FAST, -RPM_FAST, false, false, true, false); }
void KANAN_ATAS() { setTargetsRPM(0, +RPM_FAST, 0, +RPM_FAST, true, true, false, false); }
void KANAN_BAWAH() { setTargetsRPM(-RPM_FAST, 0, -RPM_FAST, 0, false, true, false, true); }
void KIRI_ATAS() { setTargetsRPM(+RPM_FAST, 0, +RPM_FAST, 0, true, false, true, false); }
void KIRI_BAWAH() { setTargetsRPM(0, -RPM_FAST, 0, -RPM_FAST, false, false, true, true); }

// ====== CETAK RPM ======
void updateAndPrintRPM() {
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint < SPEED_REPORT_MS) return;
    lastPrint = now;
    if (targetRPM[0] != 0) { // Hanya print kalau bergerak supaya serial bersih
        Serial.print("M1:"); Serial.print((int)rpmEMA[0]);
        Serial.print(" M2:"); Serial.print((int)rpmEMA[1]);
        Serial.print(" M3:"); Serial.print((int)rpmEMA[2]);
        Serial.print(" M4:"); Serial.println((int)rpmEMA[3]);
    }
}

// ====== SETUP ======
void setup()
{
    // 1. Init Serial Debug & RS485
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RS485_RO, RS485_DI);

    // 2. Init Modbus
    pinMode(RS485_READ, OUTPUT);
    digitalWrite(RS485_READ, LOW);
    modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
    modbus.begin(SLAVE_ID, 9600);

    // 3. Init Hardware
    delay(50);
    pinMode(L_MAJU, OUTPUT); pinMode(L_KANAN, OUTPUT); pinMode(L_MUNDUR, OUTPUT); pinMode(L_KIRI, OUTPUT);
    setLED(true, true, true, true);

    pinMode(PIN_ENC1_M1, INPUT); attachInterrupt(digitalPinToInterrupt(PIN_ENC1_M1), isrEnc0, RISING);
    pinMode(PIN_ENC2_M2, INPUT); attachInterrupt(digitalPinToInterrupt(PIN_ENC2_M2), isrEnc1, RISING);
    pinMode(PIN_ENC3_M3, INPUT); attachInterrupt(digitalPinToInterrupt(PIN_ENC3_M3), isrEnc2, RISING);
    pinMode(PIN_ENC4_M4, INPUT); attachInterrupt(digitalPinToInterrupt(PIN_ENC4_M4), isrEnc3, RISING);

    for (int ch = 0; ch < 8; ch++) ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(M[0].pinFwd, M[0].chFwd); ledcAttachPin(M[0].pinRev, M[0].chRev);
    ledcAttachPin(M[1].pinFwd, M[1].chFwd); ledcAttachPin(M[1].pinRev, M[1].chRev);
    ledcAttachPin(M[2].pinFwd, M[2].chFwd); ledcAttachPin(M[2].pinRev, M[2].chRev);
    ledcAttachPin(M[3].pinFwd, M[3].chFwd); ledcAttachPin(M[3].pinRev, M[3].chRev);

    STOP_COAST();
    delay(500);
    setLED(false, false, false, false);
}

// ====== LOOP ======
void loop()
{
    // --- 1. MODBUS POLL ---
    modbus.poll();

    // --- 2. CEK MODBUS COMMAND ---
    static uint16_t lastCmd = 999;
    if (holdingRegisters[0] != lastCmd) {
        lastCmd = holdingRegisters[0];
        switch (lastCmd) {
            case 0: STOP_COAST(); break;
            case 1: MAJU(); break;
            case 2: KANAN_ATAS(); break;
            case 3: KIRI_ATAS(); break;
            case 4: MUNDUR(); break;
            case 5: KANAN_BAWAH(); break;
            case 6: KIRI_BAWAH(); break;
            case 7: KANAN(); break;
            case 8: KIRI(); break;
        }
    }

    // --- 3. SERIAL CONTROL (Backup) ---
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case '1': MAJU(); break;
            case '5': MUNDUR(); break;
            case '0': STOP_COAST(); break;
        }
    }

    // --- 4. PID CONTROL ---
    speedControllerTick();

    // --- 5. MONITORING ---
    updateAndPrintRPM();
}


// #include <Arduino.h>
// #include "HardwareSerial.h"
// #include <ModbusRTUSlave.h>

// // RS485 setup with ESp32
// #define READ 4  // Connect RE terminal with 4 of ESP
// #define RO 16
// #define DI 17
// #define SLAVE_ID 2
// ModbusRTUSlave modbus(Serial2, READ);

// const uint8_t numHoldingRegisters = 10;
// uint16_t holdingRegisters[numHoldingRegisters];

// int prox = 23;
// int pwm3 = 33;
// int dir3 = 32;
// int pwm2 = 26;
// int dir2 = 25;
// int pwm1 = 14;//27;
// int dir1 = 27;//14;
// int pwm4 = 13;//12;
// int dir4 = 12;//13;

// int led3 = 19;
// int led2 = 18;
// int led4 = 5;
// int led1 = 15;

// int setPWM1 = 0;
// int setPWM2 = 0;
// int setPWM3 = 0;
// int setPWM4 = 0;
// int dirValue1 = 0;
// int dirValue2 = 0;
// int dirValue3 = 0;
// int dirValue4 = 0;


// int rpm = 0;

// void setup() {
//   // inisialiasi modbus
//   Serial.begin(115200);
//   Serial2.begin(9600, SERIAL_8N1, RO, DI);
//   pinMode(READ, OUTPUT);
//   digitalWrite(READ, LOW);

//   modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
//   modbus.begin(SLAVE_ID, 9600);

// // insialisasi motor
//   pinMode(dir1,OUTPUT);
//   pinMode(pwm1,OUTPUT);
//   pinMode(dir2,OUTPUT);
//   pinMode(pwm2,OUTPUT);
//   pinMode(dir3,OUTPUT);
//   pinMode(pwm3,OUTPUT);
//   pinMode(dir4,OUTPUT);
//   pinMode(pwm4,OUTPUT);

//   pinMode(led1, OUTPUT);
//   pinMode(led2, OUTPUT);
//   pinMode(led3, OUTPUT);
//   pinMode(led4, OUTPUT);
//   digitalWrite(led1, 0);
//   digitalWrite(led2, 0);
//   digitalWrite(led3, 0);
//   digitalWrite(led4, 0);
//   pinMode(prox, INPUT);
//   delay(1000);
//   digitalWrite(led1, 1);
//   digitalWrite(led2, 1);
//   digitalWrite(led3, 1);
//   digitalWrite(led4, 1);
// }

// void berhenti(){
//     digitalWrite(dir1, 0);
//     analogWrite(pwm1, 0);
//     digitalWrite(dir2, 0);
//     analogWrite(pwm2, 0);
//     digitalWrite(dir3, 0);
//     analogWrite(pwm3, 0);
//     digitalWrite(dir4, 0);
//     analogWrite(pwm4, 0);

//     digitalWrite(led1, 1);
//     digitalWrite(led2, 1);
//     digitalWrite(led3, 1);
//     digitalWrite(led4, 1);
// }

// void maju(){
//   setPWM1 = 165;
//   setPWM2 = 0;
//   setPWM3 = 30;
//   setPWM4 = 160;
//   dirValue1 = 0;
//   dirValue2 = 1;
//   dirValue3 = 1;
//   dirValue4 = 0;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 0);
//     digitalWrite(led2, 1);
//     digitalWrite(led3, 1);
//     digitalWrite(led4, 1);
// }

// void kananatas(){
//     setPWM1 = 0;
//     setPWM2 = 0;
//     setPWM3 = 0;
//     setPWM4 = 160;
//     dirValue1 = 0;
//     dirValue2 = 1;
//     dirValue3 = 0;
//     dirValue4 = 0;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 0);
//     digitalWrite(led2, 1);
//     digitalWrite(led3, 1);
//     digitalWrite(led4, 0);
// }

// void kiriatas(){
//     setPWM1 = 165;
//     setPWM2 = 0;
//     setPWM3 = 30;
//     setPWM4 = 0;
//     dirValue1 = 0;
//     dirValue2 = 0;
//     dirValue3 = 1;
//     dirValue4 = 0;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 0);
//     digitalWrite(led2, 0);
//     digitalWrite(led3, 1);
//     digitalWrite(led4, 1);
//     }

// void bawah(){
//     setPWM1 = 15;
//     setPWM2 = 200;
//     setPWM3 = 225;
//     setPWM4 = 33;
//     dirValue1 = 1;
//     dirValue2 = 0;
//     dirValue3 = 0;
//     dirValue4 = 1;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 1);
//     digitalWrite(led2, 1);
//     digitalWrite(led3, 0);
//     digitalWrite(led4, 1);
//     }

//     void kananbawah(){
//     setPWM1 = 15;
//     setPWM2 = 0;
//     setPWM3 = 225;
//     setPWM4 = 255;
//     dirValue1 = 1;
//     dirValue2 = 0;
//     dirValue3 = 0;
//     dirValue4 = 1;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 1);
//     digitalWrite(led2, 0);
//     digitalWrite(led3, 0);
//     digitalWrite(led4, 1);
//     }  

//     void kiribawah(){
//     setPWM1 = 225;
//     setPWM2 = 200;
//     setPWM3 = 225;
//     setPWM4 = 33;
//     dirValue1 = 1;
//     dirValue2 = 0;
//     dirValue3 = 1;
//     dirValue4 = 1;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 1);
//     digitalWrite(led2, 1);
//     digitalWrite(led3, 0);
//     digitalWrite(led4, 0);
//     }        

//     void kanan(){
//     setPWM1 = 25;
//     setPWM2 = 0;
//     setPWM3 = 240;
//     setPWM4 = 180;
//     dirValue1 = 1;
//     dirValue2 = 1;
//     dirValue3 = 0;
//     dirValue4 = 0;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 1);
//     digitalWrite(led2, 0);
//     digitalWrite(led3, 1);
//     digitalWrite(led4, 1);

//     }

//     void kiri(){
//    setPWM1 = 255;
//     setPWM2 = 255;
//     setPWM3 = 0;
//     setPWM4 = 0;
//     dirValue1 = 0;
//     dirValue2 = 0;
//     dirValue3 = 1;
//     dirValue4 = 1;
//     digitalWrite(dir1, dirValue1);
//     analogWrite(pwm1, setPWM1);
//     digitalWrite(dir2, dirValue2);
//     analogWrite(pwm2, setPWM2);
//     digitalWrite(dir3, dirValue3);
//     analogWrite(pwm3, setPWM3);
//     digitalWrite(dir4, dirValue4);
//     analogWrite(pwm4, setPWM4);

//     digitalWrite(led1, 1);
//     digitalWrite(led2, 1);
//     digitalWrite(led3, 1);
//     digitalWrite(led4, 0);
//     }

// void loop() {
//      // Update komunikasi Modbus
//   modbus.poll();

//   // Tampilkan data hanya jika ada perubahan pada holding register
//   static uint16_t previousRegisters[numHoldingRegisters] = {0};
//   bool dataChanged = false;

//   for (int i = 0; i < numHoldingRegisters; i++) {
//     if (holdingRegisters[i] != previousRegisters[i]) {
//       previousRegisters[i] = holdingRegisters[i];
//       dataChanged = true;
//     }
//   }

//   if (dataChanged) {
//     Serial.println("Data diterima dari komputer:");
//     for (int i = 0; i < numHoldingRegisters; i++) {
//       Serial.print("Register ");
//       Serial.print(i);
//       Serial.print(": ");
//       Serial.println(holdingRegisters[i]);
//     }
//        if (holdingRegisters[0] == 0) {
//     berhenti();
//   } else if (holdingRegisters[0] == 1) {
//    maju();
//   } else if (holdingRegisters[0] == 2) {
//     kananatas();
//   } else if (holdingRegisters[0] == 3) {
//     kiriatas();
//   } else if (holdingRegisters[0] == 4) {
//     bawah();
//   } else if (holdingRegisters[0] == 5) {
//     kananbawah();
//   } else if (holdingRegisters[0] == 6) {
//     kiribawah();
//   } else if (holdingRegisters[0] == 7) {
//     kanan();
//   } else if (holdingRegisters[0] == 8) {
//      kiri();
//   } else {

//   }

//   }
// }
