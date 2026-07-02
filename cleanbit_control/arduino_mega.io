/*
 * ============================================================
 * Cleanbit – Arduino Mega 2560  (merge Nano + Nucleo STM32)
 * ============================================================
 *
 * DRIVER 1  – Trazione  (L298N)
 * ENA  → pin 8   (PWM)
 * IN1  → pin 9
 * IN2  → pin 10
 * IN3  → pin 11
 * IN4  → pin 12
 * ENB  → pin 13  (PWM)
 *
 * Encoder trazione:
 * Motore SX  : A → pin 18 (INT5),  B → pin 22
 * Motore DX  : A → pin 19 (INT4),  B → pin 24
 *
 * NOTA: nessuna inversione automatica del motore destro —
 * verificato fisicamente con test (comandi j/k/o/p) che il
 * cablaggio IN3/IN4 è già coerente con il motore sinistro.
 *
 * DRIVER 2  – Spazzola  (L298N)
 * ENA  → pin 3   (PWM)
 * ENB  → pin 2   (PWM)
 * IN1  → pin 52
 * IN2  → pin 50
 * IN3  → pin 48
 * IN4  → pin 46
 * (i due motori girano sempre allo stesso duty-cycle e direzione)
 *
 * Encoder rotativo menu:
 * CLK  → pin 4
 * DT   → pin 5
 * SW   → ignorato (rotto)
 *
 * Pulsante esterno (rosso):
 * un cavo → pin 36
 * altro   → GND
 *
 * LCD I2C  → SDA pin 20,  SCL pin 21
 *
 * MPU6050 I2C → SDA pin 20,  SCL pin 21
 *
 * Buzzer      → pin 38  (PWM tone)
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ───────────────────────── Parametri robot ──────────────────
const float TRACK_RADIUS    = 0.055f;   // raggio ruota [m]
const float TRACK_BASE      = 0.246f;   // interasse ruote [m]
const float PULSES_PER_REV  = 363.3f;   // impulsi encoder / giro
const float GEAR_RATIO      = 1.0f;
const float WHEEL_CIRC      = 2.0f * PI * TRACK_RADIUS;

// Bilanciamento ruote (trovato sperimentalmente con test motori/encoder)
const float BALANCE_L = 127.0f / 144.0f;  // ≈ 0.882
const float BALANCE_R = 1.0f;             // riferimento

// ───────────────────────── Pin driver trazione ──────────────
#define ENA_T   8
#define IN1_T   9
#define IN2_T   10
#define IN3_T   11
#define IN4_T   12
#define ENB_T   13

// ───────────────────────── Pin encoder trazione ─────────────
#define ENC_SX_A  18   // INT5
#define ENC_SX_B  22
#define ENC_DX_A  19   // INT4
#define ENC_DX_B  24

// ───────────────────────── Pin driver spazzola ──────────────
#define ENA_S   3
#define ENB_S   2
#define IN1_S   52
#define IN2_S   50
#define IN3_S   48
#define IN4_S   46

// ───────────────────────── Pin encoder rotativo menu ────────
#define ROT_CLK  4
#define ROT_DT   5
#define BTN_EXT      36
#define DEBOUNCE_MS  300

// ───────────────────────── Buzzer ───────────────────────────
#define BUZZER_PIN  38

// ───────────────────────── LCD ──────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ───────────────────────── MPU6050 ──────────────────────────
unsigned long lastImuUpdate = 0;
const unsigned long IMU_PERIOD = 50;

// ───────────────────────── Variabili encoder trazione ───────
volatile long encSX = 0;
volatile long encDX = 0;

// ───────────────────────── Odometria ────────────────────────
float x = 0.0f, y = 0.0f, th = 0.0f;
float v_lin = 0.0f, v_ang = 0.0f;

unsigned long lastOdoUpdate = 0;
const unsigned long ODO_PERIOD = 100;   // ms

// ───────────────────────── Diagnostica seriale (NUOVO) ──────
// Righe che iniziano con '#' sono commenti/debug: il bridge Python
// le ignora silenziosamente (non iniziano per O/J/I), quindi sono
// sicure da aggiungere senza rompere il protocollo ROS.
unsigned long lastEncStallCheck = 0;
const unsigned long ENC_STALL_PERIOD = 2000;  // ogni 2s
long lastCheckedSX = 0, lastCheckedDX = 0;
unsigned long imuFailCount = 0;

// ───────────────────────── Menu e Interfaccia ───────────────
typedef enum { MENU_NAV = 0, SUB_SPAZ, SUB_GYRO } MenuState;
MenuState statoMenu = MENU_NAV;

uint8_t  menuSel    = 0;
uint8_t  brushSpeed = 0;

volatile int rotDelta = 0;
uint8_t  prevSW       = HIGH;
uint32_t lastSWtick   = 0;
bool aggiornaSchermo  = true;

float imu_Gz = 0.0f, imu_Gx = 0.0f, imu_Gy = 0.0f;
float imu_Ax = 0.0f, imu_Ay = 0.0f, imu_Az = 0.0f;

unsigned long lastGyroDisplay = 0;
const unsigned long GYRO_DISPLAY_PERIOD = 500;

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("# BOOT: Mega avviato, Serial OK");   // <-- marker 1

  // ── Driver trazione ──
  pinMode(ENA_T, OUTPUT); pinMode(IN1_T, OUTPUT); pinMode(IN2_T, OUTPUT);
  pinMode(ENB_T, OUTPUT); pinMode(IN3_T, OUTPUT); pinMode(IN4_T, OUTPUT);

  // ── Encoder trazione ──
  pinMode(ENC_SX_A, INPUT_PULLUP); pinMode(ENC_SX_B, INPUT_PULLUP);
  pinMode(ENC_DX_A, INPUT_PULLUP); pinMode(ENC_DX_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_SX_A), isrSX, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DX_A), isrDX, RISING);

  // ── Driver spazzola ──
  pinMode(ENA_S, OUTPUT); pinMode(ENB_S, OUTPUT);
  pinMode(IN1_S, OUTPUT); pinMode(IN2_S, OUTPUT);
  pinMode(IN3_S, OUTPUT); pinMode(IN4_S, OUTPUT);
  digitalWrite(IN1_S, HIGH); digitalWrite(IN2_S, LOW);
  digitalWrite(IN3_S, HIGH); digitalWrite(IN4_S, LOW);
  analogWrite(ENA_S, 0); analogWrite(ENB_S, 0);

  // ── Encoder rotativo menu ──
  pinMode(ROT_CLK, INPUT_PULLUP);
  pinMode(ROT_DT,  INPUT_PULLUP);
  pinMode(BTN_EXT, INPUT_PULLUP);

  // ── Buzzer ──
  pinMode(BUZZER_PIN, OUTPUT);
  tone(BUZZER_PIN, 400, 300);
  delay(350);
  noTone(BUZZER_PIN);

  Serial.println("# BOOT: pin/interrupt configurati, avvio I2C...");  // <-- marker 2

  // ── I2C bus: LCD + MPU6050 ──
  Wire.begin();
  Wire.setWireTimeout(25000, true);  // <-- timeout 25ms: evita hang infinito sul bus I2C (Mega core >= 1.8.10)

  // ── LCD ──
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cleanbit ready");

  Serial.println("# BOOT: LCD inizializzato");   // <-- marker 3 (se non arriva, LCD blocca qui)

  // ── MPU6050 ──
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  bool imuOk = (Wire.available() && Wire.read() == 0x68);

  lcd.setCursor(0, 1);
  lcd.print(imuOk ? "IMU: OK " : "IMU: ERR");

  Serial.print("# BOOT: MPU6050 ");                          // <-- marker 4
  Serial.println(imuOk ? "OK" : "NON RISPONDE (WHO_AM_I fallito)");

  delay(1200);
  lcd.clear();
  aggiornaSchermo = true;

  Serial.println("# BOOT: setup() completato, entro in loop()");  // <-- marker 5 (se manca, hang dopo IMU)
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  readRotaryEncoder();
  handleMenu();
  updateBrushMotors();
  handleROS();
  updateOdometry();
  readImu();
}

// ═══════════════════════════════════════════════════════════════
//  Lettura Encoder & Pulsante
// ═══════════════════════════════════════════════════════════════
void readRotaryEncoder() {
  static uint8_t lastClk = HIGH;
  uint8_t clk = digitalRead(ROT_CLK);

  if (clk != lastClk && clk == LOW) {
    uint8_t dt = digitalRead(ROT_DT);
    if (dt == HIGH) rotDelta = 1;
    else rotDelta = -1;
  }
  lastClk = clk;
}

bool buttonPressed() {
  uint8_t now = digitalRead(BTN_EXT);
  uint32_t t = millis();
  if (now == LOW && prevSW == HIGH && (t - lastSWtick > DEBOUNCE_MS)) {
    lastSWtick = t;
    prevSW = now;
    return true;
  }
  if (now != prevSW) prevSW = now;
  return false;
}

// ═══════════════════════════════════════════════════════════════
//  Logica e Display Menu
// ═══════════════════════════════════════════════════════════════
void handleMenu() {
  bool tasto = buttonPressed();

  switch (statoMenu) {

    case MENU_NAV:
      if (rotDelta != 0) {
        if (rotDelta > 0) menuSel = (menuSel + 1) % 2;
        if (rotDelta < 0) menuSel = (menuSel + 1) % 2;
        rotDelta = 0;
        aggiornaSchermo = true;
      }
      if (tasto) {
        if (menuSel == 0) statoMenu = SUB_SPAZ;
        else statoMenu = SUB_GYRO;
        lcd.clear();
        aggiornaSchermo = true;
      }
      break;

    case SUB_SPAZ:
      if (rotDelta != 0) {
        if (rotDelta > 0 && brushSpeed < 10) brushSpeed++;
        if (rotDelta < 0 && brushSpeed > 0) brushSpeed--;
        rotDelta = 0;
        aggiornaSchermo = true;
      }
      if (tasto) {
        statoMenu = MENU_NAV;
        lcd.clear();
        aggiornaSchermo = true;
      }
      break;

    case SUB_GYRO:
      if (rotDelta != 0) rotDelta = 0;
      if (tasto) {
        statoMenu = MENU_NAV;
        lcd.clear();
        aggiornaSchermo = true;
      }
      if (millis() - lastGyroDisplay >= GYRO_DISPLAY_PERIOD) {
        lastGyroDisplay = millis();
        aggiornaSchermo = true;
      }
      break;
  }

  if (aggiornaSchermo) {
    switch (statoMenu) {

      case MENU_NAV:
        lcd.setCursor(0, 0);
        lcd.print(menuSel == 0 ? ">Spazzola       " : " Spazzola       ");
        lcd.setCursor(0, 1);
        lcd.print(menuSel == 1 ? ">Giroscopio     " : " Giroscopio     ");
        break;

      case SUB_SPAZ: {
        char buf[17];
        snprintf(buf, sizeof(buf), "Velocita': %2d/10", brushSpeed);
        lcd.setCursor(0, 0);
        lcd.print(buf);
        lcd.setCursor(0, 1);
        lcd.print("                ");
        break;
      }

      case SUB_GYRO: {
        char sgz[9], sgx[6], sgy[6];
        dtostrf(imu_Gz, 7, 2, sgz);
        dtostrf(imu_Gx, 5, 1, sgx);
        dtostrf(imu_Gy, 5, 1, sgy);

        char buf0[17], buf1[17];
        snprintf(buf0, sizeof(buf0), "Gz:%s d/s  ", sgz);
        snprintf(buf1, sizeof(buf1), "Gx:%s %s   ", sgx, sgy);

        lcd.setCursor(0, 0);
        lcd.print(buf0);
        lcd.setCursor(0, 1);
        lcd.print(buf1);
        break;
      }
    }
    aggiornaSchermo = false;
  }
}

// ═══════════════════════════════════════════════════════════════
//  ROS serial bridge
// ═══════════════════════════════════════════════════════════════
void handleROS() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("V")) {
      line.remove(0, 1);
      line.trim();
      int sp = line.indexOf(' ');
      if (sp > 0) {
        v_lin = line.substring(0, sp).toFloat();
        v_ang = line.substring(sp + 1).toFloat();

        float v_r = v_lin + v_ang * TRACK_BASE / 2.0f; // CARLO:  cambiato poichè girava a destra quando doveva girare a sinistra e viceversa
        float v_l = v_lin - v_ang * TRACK_BASE / 2.0f;

  
        setTractionSpeed(v_l, true);
        setTractionSpeed(v_r, false);
      }
    }
    else if (line.startsWith("B")) {
      line.remove(0, 1);
      line.trim();
      int val = line.toInt();
      brushSpeed = constrain(val, 0, 10);
      aggiornaSchermo = true;
    }
  }

  if (millis() - lastOdoUpdate >= ODO_PERIOD) {
    updateOdometryCalc();
    lastOdoUpdate = millis();

    Serial.print("O ");
    Serial.print(x, 3); Serial.print(" ");
    Serial.print(y, 3); Serial.print(" ");
    Serial.println(th, 3);

    float posL = (float)encSX / PULSES_PER_REV / GEAR_RATIO * 2.0f * PI;
    float posR = (float)encDX / PULSES_PER_REV / GEAR_RATIO * 2.0f * PI;
    Serial.print("J ");
    Serial.print(posL, 3); Serial.print(" ");
    Serial.println(posR, 3);

    // ── Diagnostica: encoder fermi da troppo tempo (NUOVO) ──
    if (millis() - lastEncStallCheck >= ENC_STALL_PERIOD) {
      lastEncStallCheck = millis();
      bool sxStuck = (encSX == lastCheckedSX);
      bool dxStuck = (encDX == lastCheckedDX);
      if (sxStuck && dxStuck) {
        Serial.println("# WARN: encoder SX e DX fermi da 2s (normale se il robot e' fermo)");
      } else if (sxStuck) {
        Serial.println("# WARN: encoder SX fermo, encoder DX si muove -> controlla motore/encoder SX");
      } else if (dxStuck) {
        Serial.println("# WARN: encoder DX fermo, encoder SX si muove -> controlla motore/encoder DX");
      }
      lastCheckedSX = encSX;
      lastCheckedDX = encDX;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  Motori trazione
// ═══════════════════════════════════════════════════════════════
void setTractionSpeed(float vel_m_s, bool leftMotor) {
  bool forward = (vel_m_s >= 0);
  int  pwm     = (int)min(abs(vel_m_s) * 255.0f / 0.5f, 255.0f);

  int enPin, in1, in2;
  if (leftMotor) {
    enPin = ENA_T; in1 = IN1_T; in2 = IN2_T;
    pwm = (int)(pwm * BALANCE_L);
  } else {
    enPin = ENB_T; in1 = IN3_T; in2 = IN4_T;
    pwm = (int)(pwm * BALANCE_R);
  }

  pwm = constrain(pwm, 0, 255);

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  analogWrite(enPin, pwm);
}

// ═══════════════════════════════════════════════════════════════
//  Motori spazzola
// ═══════════════════════════════════════════════════════════════
void updateBrushMotors() {
  int pwm = map(brushSpeed, 0, 10, 0, 255);
  analogWrite(ENA_S, pwm);
  analogWrite(ENB_S, pwm);
}

// ═══════════════════════════════════════════════════════════════
//  Odometria
// ═══════════════════════════════════════════════════════════════
void updateOdometry() {}

void updateOdometryCalc() {
  static long lastSX = 0, lastDX = 0;

  long dA = encSX - lastSX;
  long dB = encDX - lastDX;
  lastSX = encSX;
  lastDX = encDX;

  float dL = (dA / PULSES_PER_REV / GEAR_RATIO) * WHEEL_CIRC;
  float dR = -(dB / PULSES_PER_REV / GEAR_RATIO) * WHEEL_CIRC; //aggiunto un meno perchè non funzionava odometria erano sempre + 20 -20 le ruote esempio

  float dS  = (dR + dL) / 2.0f;
  float dTh = (dR - dL) / TRACK_BASE;

  x  += dS * cos(th + dTh / 2.0f);
  y  += dS * sin(th + dTh / 2.0f);
  th += dTh;

  if (th >  PI) th -= 2.0f * PI;
  if (th < -PI) th += 2.0f * PI;
}

// ═══════════════════════════════════════════════════════════════
//  MPU6050
// ═══════════════════════════════════════════════════════════════
void readImu() {
  if (millis() - lastImuUpdate < IMU_PERIOD) return;
  lastImuUpdate = millis();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  if (Wire.available() < 14) {
    // ── Diagnostica: lettura IMU fallita (NUOVO) ──
    imuFailCount++;
    if (imuFailCount % 20 == 1) {  // stampa ogni ~1s invece che ogni 50ms
      Serial.println("# WARN: lettura MPU6050 fallita (I2C non risponde con 14 byte)");
    }
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  imu_Ax = ax / 16384.0f;
  imu_Ay = ay / 16384.0f;
  imu_Az = az / 16384.0f;
  imu_Gx = gx / 131.0f;
  imu_Gy = gy / 131.0f;
  imu_Gz = gz / 131.0f;

  Serial.print("I ");
  Serial.print(imu_Ax, 3); Serial.print(" ");
  Serial.print(imu_Ay, 3); Serial.print(" ");
  Serial.print(imu_Az, 3); Serial.print(" ");
  Serial.print(imu_Gx, 3); Serial.print(" ");
  Serial.print(imu_Gy, 3); Serial.print(" ");
  Serial.println(imu_Gz, 3);
}

// ═══════════════════════════════════════════════════════════════
//  ISR encoder trazione
// ═══════════════════════════════════════════════════════════════
void isrSX() {
  encSX += (digitalRead(ENC_SX_B) == LOW) ? 1 : -1;
}

void isrDX() {
  encDX += (digitalRead(ENC_DX_B) == LOW) ? 1 : -1;
}
