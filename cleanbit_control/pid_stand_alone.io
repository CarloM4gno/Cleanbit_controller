/*
 * ============================================================
 * Cleanbit – Test standalone PID trazione + Quadrato
 * Arduino Mega 2560 — NESSUNA comunicazione ROS
 * ============================================================
 *
 * DRIVER Trazione (L298N)
 * ENA → pin 8   IN1 → 9   IN2 → 10
 * IN3 → 11  IN4 → 12  ENB → 13
 *
 * Encoder trazione:
 * SX : A → 18 (INT5), B → 22
 * DX : A → 19 (INT4), B → 24
 *
 * Encoder rotativo menu: CLK → 4, DT → 5
 * Pulsante esterno: pin 36 (altro capo a GND)
 * LCD I2C: SDA 20, SCL 21, indirizzo 0x27
 * Buzzer: pin 38
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ───────────────────────── Parametri robot ──────────────────
const float TRACK_RADIUS    = 0.055f;
const float TRACK_BASE      = 0.246f;
const float PULSES_PER_REV  = 363.3f;
const float GEAR_RATIO      = 1.0f;
const float WHEEL_CIRC      = 2.0f * PI * TRACK_RADIUS;

// ───────────────────────── Pin driver trazione ──────────────
#define ENA_T   8
#define IN1_T   9
#define IN2_T   10
#define IN3_T   11
#define IN4_T   12
#define ENB_T   13

// ───────────────────────── Pin encoder trazione ─────────────
#define ENC_SX_A  18
#define ENC_SX_B  22
#define ENC_DX_A  19
#define ENC_DX_B  24

// ───────────────────────── Pin encoder rotativo menu ────────
#define ROT_CLK  4
#define ROT_DT   5
#define BTN_EXT      36
#define DEBOUNCE_MS  300

// ───────────────────────── Buzzer ───────────────────────────
#define BUZZER_PIN  38

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ───────────────────────── Variabili encoder trazione ───────
volatile long encSX = 0;
volatile long encDX = 0;

// ───────────────────────── Odometria ────────────────────────
float x = 0.0f, y = 0.0f, th = 0.0f;
float lastDTh = 0.0f;   // ultima variazione di th (serve al test per la rotazione)

unsigned long lastOdoUpdate = 0;
const unsigned long ODO_PERIOD = 30;   // ms — stesso periodo del PID, così sono sincronizzati

// ───────────────────────── PID trazione ──────────────────────
float target_v_l = 0.0f, target_v_r = 0.0f;   // [m/s]
float measured_v_l = 0.0f, measured_v_r = 0.0f;

float Kp_pid = 800.0f;
float Ki_pid = 1500.0f;
float Kd_pid = 5.0f;

float integral_L = 0.0f, integral_R = 0.0f;
float lastError_L = 0.0f, lastError_R = 0.0f;

long lastPidEncSX = 0, lastPidEncDX = 0;

unsigned long lastPidUpdate = 0;
const unsigned long PID_PERIOD = 30;   // ms

unsigned long lastPidDebug = 0;
const unsigned long PID_DEBUG_PERIOD = 500;

// ───────────────────────── Parametri test quadrato ───────────
const float TEST_SIDE_M     = 1.00f;   // lunghezza lato [m]
const float TEST_LIN_SPEED  = 0.15f;   // velocità in linea retta [m/s]
const float TEST_ANG_SPEED  = 0.6f;    // velocità angolare in rotazione [rad/s]
const float TEST_TURN_TARGET = PI / 2.0f;   // 90°
const float TURN_TOLERANCE   = 0.03f;        // ~1.7°, per fermarsi vicino ai 90°

typedef enum { TEST_IDLE = 0, TEST_FORWARD, TEST_TURN, TEST_DONE } TestState;
TestState testState = TEST_IDLE;

uint8_t testLeg = 0;          // 0..3, quale lato del quadrato
float legStartX = 0, legStartY = 0;
float legAngleAccum = 0.0f;   // angolo accumulato durante la fase di rotazione

// ───────────────────────── Menu e Interfaccia ───────────────
typedef enum { MENU_NAV = 0, SUB_PID, SUB_TEST } MenuState;
MenuState statoMenu = MENU_NAV;

const uint8_t MENU_ITEMS = 2;   // Taratura PID, Test Quadrato
uint8_t  menuSel    = 0;

uint8_t pidField   = 0;      // 0=Kp 1=Ki 2=Kd 3=Esci
bool    pidEditing = false;

volatile int rotDelta = 0;
uint8_t  prevSW       = HIGH;
uint32_t lastSWtick   = 0;
bool aggiornaSchermo  = true;

unsigned long lastTestDisplay = 0;
const unsigned long TEST_DISPLAY_PERIOD = 150;   // refresh display durante il test

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("# BOOT: sketch test PID standalone avviato");

  pinMode(ENA_T, OUTPUT); pinMode(IN1_T, OUTPUT); pinMode(IN2_T, OUTPUT);
  pinMode(ENB_T, OUTPUT); pinMode(IN3_T, OUTPUT); pinMode(IN4_T, OUTPUT);

  pinMode(ENC_SX_A, INPUT_PULLUP); pinMode(ENC_SX_B, INPUT_PULLUP);
  pinMode(ENC_DX_A, INPUT_PULLUP); pinMode(ENC_DX_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_SX_A), isrSX, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DX_A), isrDX, RISING);

  pinMode(ROT_CLK, INPUT_PULLUP);
  pinMode(ROT_DT,  INPUT_PULLUP);
  pinMode(BTN_EXT, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  tone(BUZZER_PIN, 400, 300);
  delay(350);
  noTone(BUZZER_PIN);

  Wire.begin();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cleanbit PID");
  lcd.setCursor(0, 1);
  lcd.print("test standalone");
  delay(1000);
  lcd.clear();
  aggiornaSchermo = true;

  Serial.println("# BOOT: setup completato");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  readRotaryEncoder();
  handleMenu();
  updateTractionPID();
  updateOdometryCalc();
  updateTest();
}

// ═══════════════════════════════════════════════════════════════
//  Lettura Encoder & Pulsante menu
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

float pidStepFor(uint8_t field) {
  switch (field) {
    case 0: return 10.0f;   // Kp
    case 1: return 10.0f;   // Ki
    case 2: return 0.5f;    // Kd
  }
  return 1.0f;
}

// ═══════════════════════════════════════════════════════════════
//  Menu
// ═══════════════════════════════════════════════════════════════
void handleMenu() {
  bool tasto = buttonPressed();

  switch (statoMenu) {

    case MENU_NAV:
      if (rotDelta != 0) {
        if (rotDelta > 0) menuSel = (menuSel + 1) % MENU_ITEMS;
        if (rotDelta < 0) menuSel = (menuSel + MENU_ITEMS - 1) % MENU_ITEMS;
        rotDelta = 0;
        aggiornaSchermo = true;
      }
      if (tasto) {
        if (menuSel == 0) {
          statoMenu = SUB_PID;
          pidField = 0;
          pidEditing = false;
        } else {
          statoMenu = SUB_TEST;
          testState = TEST_IDLE;
        }
        lcd.clear();
        aggiornaSchermo = true;
      }
      break;

    case SUB_PID:
      if (rotDelta != 0) {
        if (!pidEditing) {
          if (rotDelta > 0) pidField = (pidField + 1) % 4;
          if (rotDelta < 0) pidField = (pidField + 3) % 4;
        } else {
          float step = pidStepFor(pidField) * rotDelta;
          if (pidField == 0)      { Kp_pid = max(0.0f, Kp_pid + step); }
          else if (pidField == 1) { Ki_pid = max(0.0f, Ki_pid + step); }
          else if (pidField == 2) { Kd_pid = max(0.0f, Kd_pid + step); }
        }
        rotDelta = 0;
        aggiornaSchermo = true;
      }
      if (tasto) {
        if (pidField == 3) {
          statoMenu = MENU_NAV;
          lcd.clear();
        } else {
          pidEditing = !pidEditing;
        }
        aggiornaSchermo = true;
      }
      break;

    case SUB_TEST:
      // se il test è fermo o finito, il pulsante lo avvia/riavvia
      if (tasto) {
        if (testState == TEST_IDLE || testState == TEST_DONE) {
          startTest();
        } else {
          // test in corso -> il pulsante lo interrompe
          abortTest();
        }
        aggiornaSchermo = true;
      }
      // rotazione encoder in questa schermata = torna al menu (solo se non in corso)
      if (rotDelta != 0) {
        if (testState == TEST_IDLE || testState == TEST_DONE) {
          statoMenu = MENU_NAV;
          lcd.clear();
        }
        rotDelta = 0;
        aggiornaSchermo = true;
      }
      break;
  }

  if (aggiornaSchermo && statoMenu != SUB_TEST) {
    drawMenuScreen();
    aggiornaSchermo = false;
  }

  // il test ridisegna la sua schermata a un ritmo suo (vedi updateTest)
}

void drawMenuScreen() {
  switch (statoMenu) {

    case MENU_NAV: {
      const char* names[MENU_ITEMS] = { "Taratura PID", "Test Quadrato" };
      uint8_t next = (menuSel + 1) % MENU_ITEMS;
      char l0[17], l1[17];
      snprintf(l0, sizeof(l0), ">%-15s", names[menuSel]);
      snprintf(l1, sizeof(l1), " %-15s", names[next]);
      lcd.setCursor(0, 0); lcd.print(l0);
      lcd.setCursor(0, 1); lcd.print(l1);
      break;
    }

    case SUB_PID: {
      if (pidField == 3) {
        lcd.setCursor(0, 0);
        lcd.print(pidEditing ? " Torna indietro " : ">Torna indietro ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
      } else {
        const char* names[3] = { "Kp", "Ki", "Kd" };
        float val = (pidField == 0) ? Kp_pid : (pidField == 1) ? Ki_pid : Kd_pid;
        char sval[10];
        dtostrf(val, 8, 2, sval);

        char l0[17], l1[17];
        snprintf(l0, sizeof(l0), "%s%-4s%s", pidEditing ? "*" : ">", names[pidField], pidEditing ? " EDIT   " : "        ");
        snprintf(l1, sizeof(l1), "=%s", sval);

        lcd.setCursor(0, 0); lcd.print(l0);
        lcd.setCursor(0, 1); lcd.print(l1);
      }
      break;
    }

    default: break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  Test Quadrato
// ═══════════════════════════════════════════════════════════════
void startTest() {
  x = 0; y = 0; th = 0;   // azzera odometria a inizio test
  testLeg = 0;
  legStartX = 0; legStartY = 0;
  legAngleAccum = 0.0f;
  integral_L = integral_R = 0.0f;
  lastError_L = lastError_R = 0.0f;
  testState = TEST_FORWARD;
  target_v_l = TEST_LIN_SPEED;
  target_v_r = TEST_LIN_SPEED;
  tone(BUZZER_PIN, 800, 100);
  Serial.println("# TEST: avviato");
}

void abortTest() {
  testState = TEST_IDLE;
  target_v_l = 0; target_v_r = 0;
  integral_L = integral_R = 0.0f;
  tone(BUZZER_PIN, 300, 300);
  Serial.println("# TEST: interrotto manualmente");
}

void updateTest() {
  if (statoMenu != SUB_TEST) return;

  switch (testState) {

    case TEST_FORWARD: {
      float dist = hypot(x - legStartX, y - legStartY);
      if (dist >= TEST_SIDE_M) {
        // fine lato: passa a rotazione di 90° a destra (clockwise -> v_ang negativo)
        legAngleAccum = 0.0f;
        target_v_l = -(TEST_ANG_SPEED * TRACK_BASE / 2.0f);
        target_v_r =  (TEST_ANG_SPEED * TRACK_BASE / 2.0f);
        testState = TEST_TURN;
        tone(BUZZER_PIN, 800, 80);
      }
      break;
    }

    case TEST_TURN: {
      legAngleAccum += fabs(lastDTh);
      if (legAngleAccum >= TEST_TURN_TARGET - TURN_TOLERANCE) {
        testLeg++;
        if (testLeg >= 4) {
          // quadrato completo
          target_v_l = 0; target_v_r = 0;
          testState = TEST_DONE;
          tone(BUZZER_PIN, 1200, 400);
          Serial.println("# TEST: quadrato completato");
        } else {
          // prossimo lato
          legStartX = x; legStartY = y;
          target_v_l = TEST_LIN_SPEED;
          target_v_r = TEST_LIN_SPEED;
          testState = TEST_FORWARD;
          tone(BUZZER_PIN, 800, 80);
        }
      }
      break;
    }

    default: break;   // TEST_IDLE, TEST_DONE: fermo, aspetta il pulsante
  }

  if (millis() - lastTestDisplay >= TEST_DISPLAY_PERIOD) {
    lastTestDisplay = millis();
    drawTestScreen();
  }
}

void drawTestScreen() {
  char l0[17], l1[17];

  switch (testState) {
    case TEST_IDLE:
      snprintf(l0, sizeof(l0), "Test Quadrato   ");
      snprintf(l1, sizeof(l1), "Premi: avvia    ");
      break;

    case TEST_FORWARD: {
      float dist = hypot(x - legStartX, y - legStartY);
      snprintf(l0, sizeof(l0), "Lato %d/4 AVANTI", testLeg + 1);
      char sd[8]; dtostrf(dist, 4, 2, sd);
      snprintf(l1, sizeof(l1), "%s / %.2fm      ", sd, TEST_SIDE_M);
      break;
    }

    case TEST_TURN: {
      float deg = legAngleAccum * 180.0f / PI;
      snprintf(l0, sizeof(l0), "Lato %d/4 GIRA  ", testLeg + 1);
      char sdeg[6]; dtostrf(deg, 4, 0, sdeg);
      snprintf(l1, sizeof(l1), "%s / 90 gradi   ", sdeg);
      break;
    }

    case TEST_DONE:
      snprintf(l0, sizeof(l0), "Quadrato OK!    ");
      snprintf(l1, sizeof(l1), "Premi: ripeti   ");
      break;
  }

  lcd.setCursor(0, 0); lcd.print(l0);
  lcd.setCursor(0, 1); lcd.print(l1);
}

// ═══════════════════════════════════════════════════════════════
//  PID trazione
// ═══════════════════════════════════════════════════════════════
void updateTractionPID() {
  unsigned long now = millis();
  if (now - lastPidUpdate < PID_PERIOD) return;
  float dt = (now - lastPidUpdate) / 1000.0f;
  lastPidUpdate = now;

  long dSX = encSX - lastPidEncSX;
  long dDX = encDX - lastPidEncDX;
  lastPidEncSX = encSX;
  lastPidEncDX = encDX;

  measured_v_l = (dSX / PULSES_PER_REV / GEAR_RATIO) * WHEEL_CIRC / dt;
  measured_v_r = -(dDX / PULSES_PER_REV / GEAR_RATIO) * WHEEL_CIRC / dt;

  float pwm_l = computePID(target_v_l, measured_v_l, integral_L, lastError_L, dt);
  float pwm_r = computePID(target_v_r, measured_v_r, integral_R, lastError_R, dt);

  applyMotorPWM(pwm_l, true);
  applyMotorPWM(pwm_r, false);

  if (now - lastPidDebug >= PID_DEBUG_PERIOD) {
    lastPidDebug = now;
    Serial.print("# PID L target=");
    Serial.print(target_v_l, 3);
    Serial.print(" meas=");
    Serial.print(measured_v_l, 3);
    Serial.print("  R target=");
    Serial.print(target_v_r, 3);
    Serial.print(" meas=");
    Serial.println(measured_v_r, 3);
  }
}

float computePID(float target, float measured, float &integral, float &lastError, float dt) {
  if (fabs(target) < 0.001f) {
    integral = 0.0f;
    lastError = 0.0f;
    return 0.0f;
  }

  float error = target - measured;

  integral += error * dt;
  integral = constrain(integral, -100.0f, 100.0f);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float feedforward = target / 0.5f * 255.0f;

  float output = feedforward + Kp_pid * error + Ki_pid * integral + Kd_pid * derivative;

  return constrain(output, -255.0f, 255.0f);
}

void applyMotorPWM(float pwmSigned, bool leftMotor) {
  bool forward = (pwmSigned >= 0);
  int  pwm     = (int)min(fabs(pwmSigned), 255.0f);

  int enPin, in1, in2;
  if (leftMotor) {
    enPin = ENA_T; in1 = IN1_T; in2 = IN2_T;
  } else {
    enPin = ENB_T; in1 = IN3_T; in2 = IN4_T;
  }

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  analogWrite(enPin, pwm);
}

// ═══════════════════════════════════════════════════════════════
//  Odometria
// ═══════════════════════════════════════════════════════════════
void updateOdometryCalc() {
  unsigned long now = millis();
  if (now - lastOdoUpdate < ODO_PERIOD) return;
  lastOdoUpdate = now;

  static long lastSX = 0, lastDX = 0;

  long dA = encSX - lastSX;
  long dB = encDX - lastDX;
  lastSX = encSX;
  lastDX = encDX;

  float dL = (dA / PULSES_PER_REV / GEAR_RATIO) * WHEEL_CIRC;
  float dR = -(dB / PULSES_PER_REV / GEAR_RATIO) * WHEEL_CIRC;

  float dS  = (dR + dL) / 2.0f;
  float dTh = (dR - dL) / TRACK_BASE;
  lastDTh = dTh;

  x  += dS * cos(th + dTh / 2.0f);
  y  += dS * sin(th + dTh / 2.0f);
  th += dTh;

  if (th >  PI) th -= 2.0f * PI;
  if (th < -PI) th += 2.0f * PI;
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
