// CARA PAKAI
// Ketik K150    (Misalkan mau set rpm 150 ke masing masing roda)
// Tunggu Hingga Kalibrasi Selesai
// Done
// 0 Untuk Mematikan Semua
// Sudah Membaca 4 RPM dan auto Adjust PWM jika terdapat perubahan pembacaan rpm
// Jika ada lonjakan tidak normal diambil data terakhir yang normal untuk di print di serial monitor

// --- DEFINISI PIN ---
int prox = 23;
int pwm3 = 33; int dir3 = 32;
int pwm4 = 26; int dir4 = 25;
int pwm2 = 14; int dir2 = 27;
int pwm1 = 13; int dir1 = 12;

int led3 = 19; int led2 = 18;
int led4 = 5;  int led1 = 15;

const int encoderPin1 = 36;
const int encoderPin2 = 35;
const int encoderPin3 = 34;
const int encoderPin4 = 39;

// --- VARIABEL SISTEM ---
String dataTerima = "";

int basePWM[5]   = {0, 0, 0, 0, 0};
int currentPWM[5]= {0, 0, 0, 0, 0};
int memoriRPM[5] = {0, 0, 0, 0, 0}; // Menyimpan angka valid terakhir

// --- STATUS SISTEM ---
enum State { STANDBY, CALIBRATING, RUNNING };
State systemState = STANDBY;

int targetRPM = 0;
int calibMotor = 1;     
int calibStep = 0;      
unsigned long calibTimer = 0;
unsigned long lastPrintTimer = 0; 
int testPWM = 255;

int toleransiMasuk = 2; 
int toleransiUji = 5;   

void setup() {
  pinMode(dir1, OUTPUT); pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT); pinMode(pwm2, OUTPUT);
  pinMode(dir3, OUTPUT); pinMode(pwm3, OUTPUT);
  pinMode(dir4, OUTPUT); pinMode(pwm4, OUTPUT);

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP);
  pinMode(encoderPin4, INPUT_PULLUP);

  pinMode(led1, OUTPUT); pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT); pinMode(led4, OUTPUT);
  
  stopAllMotors();
  Serial.begin(115200);
  Serial.println("\n=== SISTEM KALIBRASI & RUNNING BERGILIR ===");
  Serial.println("Ketik 'K150' untuk mulai kalibrasi target 150 RPM.");
  Serial.println("Ketik '0' untuk berhenti darurat (STANDBY).\n");
}

void loop() {
  if (Serial.available() > 0) {
    dataTerima = Serial.readStringUntil('\n');
    dataTerima.trim();
    prosesPerintah();
  }

  if (systemState == CALIBRATING) {
    jalankanKalibrasi();
  } 
  else if (systemState == RUNNING) {
    jalankanOperasi();
  }
}

// ==========================================
// FUNGSI BACA RPM 
// ==========================================
int getRPM(int pinEncoder) {
  // Timeout 800ms cukup untuk membaca minimal sekitar 75 RPM
  unsigned long pulsa = pulseIn(pinEncoder, LOW, 800000); 
  if (pulsa == 0) return 0; 
  int rpm = 30000000.0 / pulsa;
  return rpm;
}

void prosesPerintah() {
  if (dataTerima.startsWith("K")) {
    targetRPM = dataTerima.substring(1).toInt();
    if (targetRPM > 0) {
      systemState = CALIBRATING;
      calibMotor = 1; 
      calibStep = 0;   
      Serial.printf("\n[MEMULAI KALIBRASI] Target: %d RPM\n", targetRPM);
    }
  } 
  else if (dataTerima == "0") {
    systemState = STANDBY;
    stopAllMotors();
    Serial.println("\n[STANDBY] Semua motor dihentikan.");
  }
}

void jalankanKalibrasi() {
  int pinSekarang;
  if (calibMotor == 1) pinSekarang = encoderPin1;
  else if (calibMotor == 2) pinSekarang = encoderPin2;
  else if (calibMotor == 3) pinSekarang = encoderPin3;
  else pinSekarang = encoderPin4;

  if (calibStep == 0) {
    testPWM = 255;
    setMotorPWM(calibMotor, testPWM);
    Serial.printf("\n-> [M%d] Cek Top Speed (PWM 255). Tunggu 3 detik...\n", calibMotor);
    calibTimer = millis();
    calibStep = 1;
  }
  
  else if (calibStep == 1) {
    if (millis() - calibTimer >= 3000) calibStep = 2; 
  }

  else if (calibStep == 2) {
    int maxRPM = getRPM(pinSekarang);
    if (maxRPM == 0 || maxRPM > 1000) return; 

    float rasio = (float)targetRPM / (float)maxRPM;
    testPWM = rasio * 255.0;
    
    testPWM = constrain(testPWM, 30, 255);
    setMotorPWM(calibMotor, testPWM);

    Serial.printf("   [M%d] Max: %d RPM. Lompat Prediksi ke PWM %d. Tunggu 2 detik...\n", calibMotor, maxRPM, testPWM);
    calibTimer = millis();
    calibStep = 3; 
  }

  else if (calibStep == 3) {
    if (millis() - calibTimer >= 2000) calibStep = 4;
  }

  else if (calibStep == 4) {
    int aktualRPM = getRPM(pinSekarang);
    
    if (aktualRPM == 0 || aktualRPM > 1000) return; 

    int error = targetRPM - aktualRPM;
    int selisih = abs(error);

    if (selisih <= toleransiMasuk) {
      Serial.printf("   [M%d] Sangat Pas! (%d RPM). Memulai Uji Stabil 10 Detik...\n", calibMotor, aktualRPM);
      calibTimer = millis();
      lastPrintTimer = millis();
      calibStep = 5;
    } 
    else {
      int loncatan = 2; 
      
      if (selisih >= 50) loncatan = 20;
      else if (selisih >= 20) loncatan = 10;
      else if (selisih > 5) loncatan = 4; 
      else loncatan = 2; 

      if (error < 0) testPWM -= loncatan;      
      else testPWM += loncatan; 

      testPWM = constrain(testPWM, 30, 255);
      setMotorPWM(calibMotor, testPWM);
      
      Serial.printf("   [M%d] Aktual: %d RPM (Selisih %d). Koreksi +/-%d -> PWM %d...\n", calibMotor, aktualRPM, selisih, loncatan, testPWM);
      calibTimer = millis();
      calibStep = 3; 
    }
  }

  else if (calibStep == 5) {
    int aktualRPM = getRPM(pinSekarang);
    if (aktualRPM == 0 || aktualRPM > 1000) return; 

    unsigned long waktuBertahan = millis() - calibTimer;

    if (millis() - lastPrintTimer >= 1000) {
      Serial.printf("      Tes M%d -> %d RPM... (%d / 10 dtk)\n", calibMotor, aktualRPM, waktuBertahan/1000);
      lastPrintTimer = millis();
    }

    int error = targetRPM - aktualRPM;

    if (abs(error) > toleransiUji) {
      Serial.printf("   [M%d] GAGAL UJI! Melenceng jauh ke %d RPM. Koreksi ulang...\n", calibMotor, aktualRPM);
      calibStep = 4; 
    } 
    else {
      if (waktuBertahan >= 10000) { 
        basePWM[calibMotor] = testPWM;
        Serial.printf("\n=> MOTOR %d LULUS UJI 10 DETIK! (Disimpan PWM: %d)\n", calibMotor, testPWM);
        
        setMotorPWM(calibMotor, 0); 
        calibMotor++;               
        calibStep = 0;              
        delay(1500); 

        if (calibMotor > 4) {
          Serial.println("\n==========================================");
          Serial.println("       SEMUA MOTOR SELESAI KALIBRASI      ");
          Serial.println("==========================================");
          Serial.printf(" -> Motor 1: PWM %d\n", basePWM[1]);
          Serial.printf(" -> Motor 2: PWM %d\n", basePWM[2]);
          Serial.printf(" -> Motor 3: PWM %d\n", basePWM[3]);
          Serial.printf(" -> Motor 4: PWM %d\n", basePWM[4]);
          Serial.println("==========================================\n");
          
          Serial.println("MASUK MODE OPERASI SERENTAK...");
          for(int i=1; i<=4; i++) {
            currentPWM[i] = basePWM[i];
            memoriRPM[i] = targetRPM; // Anggap start awal sesuai target
            setMotorPWM(i, currentPWM[i]);
          }
          systemState = RUNNING;
        }
      }
    }
  }
}

// ==========================================
// FUNGSI OPERASI (BACA BERGILIR + MEMORI)
// ==========================================
void jalankanOperasi() {
  Serial.print("[RUNNING] ");
  
  for (int i = 1; i <= 4; i++) {
    int pinSekarang = (i == 1) ? encoderPin1 : (i == 2) ? encoderPin2 : (i == 3) ? encoderPin3 : encoderPin4;
    
    // Baca sensor
    int aktualRPM = getRPM(pinSekarang); 
    
    // Jika data ngaco (Noise atau Missed Pulse)
    if (aktualRPM == 0 || aktualRPM > 1000) {
      // JANGAN ubah PWM, JANGAN ubah Memori. Biarkan motor jalan stabil.
      Serial.printf("M%d: %d RPM (PWM:%d) | ", i, memoriRPM[i], currentPWM[i]);
    } 
    // Jika data valid (Normal)
    else {
      memoriRPM[i] = aktualRPM; // Simpan angka baru ke memori
      int error = targetRPM - aktualRPM;

      // Koreksi sangat halus
      if (error > 2) currentPWM[i]++;
      else if (error < -2) currentPWM[i]--;

      currentPWM[i] = constrain(currentPWM[i], 0, 255);
      Serial.printf("M%d: %d RPM (PWM:%d) | ", i, aktualRPM, currentPWM[i]);
    }
    
    // Kirim sinyal PWM ke motor
    setMotorPWM(i, currentPWM[i]);
    
    // Beri jeda 10 milidetik agar kelistrikan stabil sebelum membaca motor berikutnya
    delay(10); 
  }
  Serial.println(); 
}

// --- FUNGSI KONTROL HARDWARE ---
void stopAllMotors() {
  setMotorPWM(1, 0); setMotorPWM(2, 0);
  setMotorPWM(3, 0); setMotorPWM(4, 0);
  digitalWrite(led1, 1); digitalWrite(led2, 1); 
  digitalWrite(led3, 1); digitalWrite(led4, 1);
}

void setMotorPWM(int num, int pwmValue) {
  if (num == 1) { analogWrite(pwm1, pwmValue); digitalWrite(dir1, 0); digitalWrite(led1, pwmValue == 0 ? 1 : 0); }
  if (num == 2) { analogWrite(pwm2, pwmValue); digitalWrite(dir2, 0); digitalWrite(led2, pwmValue == 0 ? 1 : 0); }
  if (num == 3) { analogWrite(pwm3, pwmValue); digitalWrite(dir3, 0); digitalWrite(led3, pwmValue == 0 ? 1 : 0); }
  if (num == 4) { analogWrite(pwm4, pwmValue); digitalWrite(dir4, 0); digitalWrite(led4, pwmValue == 0 ? 1 : 0); }
}
