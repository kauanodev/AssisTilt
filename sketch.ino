/*

  AssisTilt — Controlador assistivo por inclinação e gesto de aproximação

  Disciplina: SMD0114 Computação Física (UFC / IUVI / SMD) — 2026.2

  Autor: Victor Kauan Xavier Alcântara (538023)

  Placa ........ Arduino Uno (simulado no Wokwi)

  Barramento I2C (Mestre = Uno | Escravos = MPU6050 0x68, LCD 16x2 0x27)

  Sensores ..... MPU6050 (I2C), LDR (analógico, média móvel + calibração), HC-SR04 (digital, ISR)

  Atuadores .... Servo (D10), LED RGB PWM (D5/D6/D3), Buzzer (D8), LCD 16x2 I2C

  Serial ....... 115200 baud, 8N1, mensagens terminadas em '\n'

  Telemetria (Uno -> App):  uma linha JSON a cada 100 ms

  Comandos   (App -> Uno):  "SERVO:90"  "RGB:255,120,0"  "BUZZ:1200,200"

                            "MODE:AUTO" "MODE:MANUAL"  "LIGHT:1"  "PING"

  Regra de projeto: nenhum delay() no loop. Toda temporização usa millis()/micros().

  Nota de timers (Uno): Servo usa Timer1 (inutiliza PWM em D9/D10);

                        tone() usa Timer2 (interfere no PWM de D3/D11 enquanto toca).

*/
#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

// ---------------------------------------------------------------- Pinos

const uint8_t PIN_LDR    = A0;

const uint8_t PIN_ECHO   = 2;   // INT0 (interrupção externa)

const uint8_t PIN_TRIG   = 4;

const uint8_t PIN_RGB_R  = 5;   // PWM Timer0

const uint8_t PIN_RGB_G  = 6;   // PWM Timer0

const uint8_t PIN_RGB_B  = 3;   // PWM Timer2

const uint8_t PIN_BUZZER = 8;

const uint8_t PIN_SERVO  = 10;

// ---------------------------------------------------------------- Periféricos I2C

const uint8_t MPU_ADDR = 0x68;

LiquidCrystal_I2C lcd(0x27, 16, 2);

Servo servo;

// ---------------------------------------------------------------- Períodos (ms)

const unsigned long T_LDR = 20, T_IMU = 20, T_ULTRA = 100, T_SERVO = 20,

                    T_TELEM = 100, T_LCD = 250;

// ---------------------------------------------------------------- Limiares do projeto

const float    HAND_NEAR_CM    = 15.0;   // mão "dentro" do campo do gesto

const float    HAND_FAR_CM     = 20.0;   // histerese para liberar o gesto

const uint16_t HAND_HOLD_MS    = 300;    // mão deve permanecer 300 ms

const uint16_t GESTURE_LOCK_MS = 1000;   // intervalo mínimo entre acionamentos

const float    ALERT_ON_DEG    = 60.0;   // tombamento: |roll| ou |pitch| acima disso

const float    ALERT_OFF_DEG   = 45.0;   // histerese para sair do alerta

const uint16_t ALERT_HOLD_MS   = 500;

// ---------------------------------------------------------------- Estado geral

enum Mode { AUTO, MANUAL };

Mode mode = AUTO;

bool lightOn = true;

bool alertActive = false;

// ---------------------------------------------------------------- LDR: média móvel + calibração

const uint8_t N_AVG = 10;

int  ldrBuf[N_AVG];

uint8_t ldrIdx = 0;

long ldrSum = 0;

int   ldrRaw = 0;

float ldrAvg = 0;

float lux = 0;

// Modelo do módulo LDR do Wokwi (documentado): R = 2k*V/(1-V/5); lux = (RL10*1e3*10^g / R)^(1/g)

const float GAMMA = 0.7, RL10 = 50.0;

float adcToLux(float adc) {

  adc = constrain(adc, 1.0, 1022.0);           // evita divisão por zero nos extremos

  float v = adc / 1024.0 * 5.0;

  float r = 2000.0 * v / (1.0 - v / 5.0);

  return pow(RL10 * 1000.0 * pow(10, GAMMA) / r, 1.0 / GAMMA);

}

void sampleLdr() {

  int raw = analogRead(PIN_LDR);

  ldrSum -= ldrBuf[ldrIdx];

  ldrBuf[ldrIdx] = raw;

  ldrSum += raw;

  ldrIdx = (ldrIdx + 1) % N_AVG;

  ldrRaw = raw;

  ldrAvg = ldrSum / (float)N_AVG;

  lux = adcToLux(ldrAvg);

}

// ---------------------------------------------------------------- MPU6050 (I2C, registradores diretos)

float ax, ay, az, gx, gy, gz;

float roll = 0, pitch = 0;

bool imuOk = false;

unsigned long lastImuUs = 0;

bool imuInit() {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x6B);                 // PWR_MGMT_1

  Wire.write(0x00);                 // sai do modo sleep

  return Wire.endTransmission() == 0;

}

bool imuRead() {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);                 // ACCEL_XOUT_H

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  int16_t v[7];

  for (uint8_t i = 0; i < 7; i++) { // ax ay az temp gx gy gz (cada um 2 bytes, MSB primeiro)

    uint8_t hi = Wire.read();

    uint8_t lo = Wire.read();

    v[i] = (int16_t)((hi << 8) | lo);

  }

  ax = v[0] / 16384.0; ay = v[1] / 16384.0; az = v[2] / 16384.0;   // ±2 g

  gx = v[4] / 131.0;   gy = v[5] / 131.0;   gz = v[6] / 131.0;     // ±250 °/s

  return true;

}

void updateAttitude() {

  unsigned long nowUs = micros();

  float dt = (nowUs - lastImuUs) / 1e6;

  lastImuUs = nowUs;

  if (!imuRead()) { imuOk = false; return; }

  imuOk = true;

  float accRoll  = atan2(ay, az) * 180.0 / PI;

  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Filtro complementar: giroscópio (rápido) + acelerômetro (sem deriva)

  roll  = 0.96 * (roll  + gx * dt) + 0.04 * accRoll;

  pitch = 0.96 * (pitch + gy * dt) + 0.04 * accPitch;

}

// ---------------------------------------------------------------- HC-SR04 sem bloqueio (ISR no pino ECHO)

volatile unsigned long echoStartUs = 0, echoWidthUs = 0;

volatile bool echoDone = false;

float distCm = -1;

bool ultraWaiting = false;

unsigned long lastUltra = 0, trigAt = 0;

void onEcho() {

  if (digitalRead(PIN_ECHO)) echoStartUs = micros();

  else { echoWidthUs = micros() - echoStartUs; echoDone = true; }

}

void serviceUltrasonic(unsigned long now) {

  if (echoDone) {

    noInterrupts(); unsigned long w = echoWidthUs; echoDone = false; interrupts();

    distCm = w / 58.0;                         // d[cm] = t[µs] / 58

    if (distCm > 400) distCm = -1;

    ultraWaiting = false;

  } else if (ultraWaiting && now - trigAt > 30) {

    distCm = -1;                               // sem eco em 30 ms: fora de alcance

    ultraWaiting = false;

  }

  if (!ultraWaiting && now - lastUltra >= T_ULTRA) {

    lastUltra = now;

    digitalWrite(PIN_TRIG, HIGH);

    delayMicroseconds(10);                     // pulso de disparo de 10 µs (exigido pelo sensor)

    digitalWrite(PIN_TRIG, LOW);

    ultraWaiting = true;

    trigAt = now;

  }

}

// ---------------------------------------------------------------- Gesto de aproximação (liga/desliga luz)

unsigned long handSince = 0, lastToggle = 0;

bool handLatched = false;

void serviceGesture(unsigned long now) {

  bool near = (distCm > 0 && distCm < HAND_NEAR_CM);

  bool far  = (distCm < 0 || distCm > HAND_FAR_CM);

  if (near) {

    if (handSince == 0) handSince = now;

    if (!handLatched && now - handSince >= HAND_HOLD_MS && now - lastToggle >= GESTURE_LOCK_MS) {

      lightOn = !lightOn;

      handLatched = true;

      lastToggle = now;

      tone(PIN_BUZZER, lightOn ? 1500 : 800, 80);   // feedback sonoro distinto

    }

  } else if (far) {

    handSince = 0;

    handLatched = false;

  }

}

// ---------------------------------------------------------------- Alerta de tombamento

unsigned long alertSince = 0, lastBlink = 0;

bool blinkOn = false;

void serviceAlert(unsigned long now) {

  float m = max(fabs(roll), fabs(pitch));

  if (!alertActive) {

    if (m > ALERT_ON_DEG) { if (alertSince == 0) alertSince = now; if (now - alertSince >= ALERT_HOLD_MS) alertActive = true; }

    else alertSince = 0;

  } else if (m < ALERT_OFF_DEG) { alertActive = false; alertSince = 0; }

  if (alertActive && now - lastBlink >= 250) {

    lastBlink = now;

    blinkOn = !blinkOn;

    if (blinkOn) tone(PIN_BUZZER, 1800, 200);

  }

}

// ---------------------------------------------------------------- Atuadores

int servoTarget = 90, servoPos = 90, servoManual = 90;

uint8_t rgbManual[3] = {255, 255, 255};

uint8_t rgbOut[3] = {0, 0, 0};

void serviceActuators(unsigned long now) {

  static unsigned long lastServo = 0;

  if (now - lastServo >= T_SERVO) {

    lastServo = now;

    servoTarget = (mode == AUTO) ? map((long)constrain(roll, -45, 45), -45, 45, 0, 180) : servoManual;

    if (servoPos < servoTarget) servoPos = min(servoPos + 3, servoTarget);   // rampa suave

    else if (servoPos > servoTarget) servoPos = max(servoPos - 3, servoTarget);

    servo.write(servoPos);

  }

  // LED RGB: prioridade alerta > desligado > automático/manual

  if (alertActive)      { rgbOut[0] = blinkOn ? 255 : 0; rgbOut[1] = 0; rgbOut[2] = 0; }

  else if (!lightOn)    { rgbOut[0] = rgbOut[1] = rgbOut[2] = 0; }

  else if (mode == AUTO) {                       // luz de cabeceira: mais escuro -> mais brilho

    float dark = constrain(ldrAvg / 1023.0, 0.1, 1.0);   // no módulo Wokwi, ADC alto = pouco claro

    rgbOut[0] = 255 * dark; rgbOut[1] = 140 * dark; rgbOut[2] = 40 * dark;   // branco quente

  } else { for (uint8_t i = 0; i < 3; i++) rgbOut[i] = rgbManual[i]; }

  analogWrite(PIN_RGB_R, rgbOut[0]);

  analogWrite(PIN_RGB_G, rgbOut[1]);

  analogWrite(PIN_RGB_B, rgbOut[2]);

}

// ---------------------------------------------------------------- LCD 16x2 (I2C)

void serviceLcd() {

  char l0[17], l1[17], t[17];

  if (alertActive) {

    snprintf(l0, sizeof l0, "%-16s", "!! ALERTA: TOMBO");

    snprintf(l1, sizeof l1, "R%+04d P%+04d", (int)roll, (int)pitch);

    snprintf(t, sizeof t, "%-16s", l1); strcpy(l1, t);

  } else {

    snprintf(t, sizeof t, "R%+04d P%+04d %c%c", (int)roll, (int)pitch, mode == AUTO ? 'A' : 'M', lightOn ? '*' : '-');

    snprintf(l0, sizeof l0, "%-16s", t);

    int luxI = (int)constrain(lux, 0, 9999);

    if (distCm > 0) snprintf(t, sizeof t, "L%4dlx D%3dcm", luxI, (int)distCm);

    else            snprintf(t, sizeof t, "L%4dlx D --cm", luxI);

    snprintf(l1, sizeof l1, "%-16s", t);

  }

  lcd.setCursor(0, 0); lcd.print(l0);

  lcd.setCursor(0, 1); lcd.print(l1);

}

// ---------------------------------------------------------------- Telemetria JSON (Uno -> App)

void sendTelemetry(unsigned long now) {

  Serial.print(F("{\"t\":"));      Serial.print(now);

  Serial.print(F(",\"roll\":"));   Serial.print(roll, 1);

  Serial.print(F(",\"pitch\":"));  Serial.print(pitch, 1);

  Serial.print(F(",\"ldr\":"));    Serial.print(ldrRaw);

  Serial.print(F(",\"avg\":"));    Serial.print(ldrAvg, 1);

  Serial.print(F(",\"lux\":"));    Serial.print(lux, 1);

  Serial.print(F(",\"dist\":"));   Serial.print(distCm, 1);

  Serial.print(F(",\"servo\":"));  Serial.print(servoPos);

  Serial.print(F(",\"rgb\":["));   Serial.print(rgbOut[0]); Serial.print(',');

                                   Serial.print(rgbOut[1]); Serial.print(',');

                                   Serial.print(rgbOut[2]); Serial.print(']');

  Serial.print(F(",\"mode\":\"")); Serial.print(mode == AUTO ? F("AUTO") : F("MANUAL")); Serial.print('"');

  Serial.print(F(",\"light\":"));  Serial.print(lightOn ? 1 : 0);

  Serial.print(F(",\"alert\":"));  Serial.print(alertActive ? 1 : 0);

  Serial.print(F(",\"imu\":"));    Serial.print(imuOk ? 1 : 0);

  Serial.println('}');

}

void ack(const char* cmd, bool ok) {

  Serial.print(F("{\"ack\":\"")); Serial.print(cmd);

  Serial.print(F("\",\"ok\":"));  Serial.print(ok ? 1 : 0);

  Serial.println('}');

}

// ---------------------------------------------------------------- Comandos (App -> Uno), sem bloqueio

char rxBuf[40];

uint8_t rxLen = 0;

void handleCommand(char* line) {

  char* arg = "";

  char* sep = strchr(line, ':');

  if (sep) { *sep = 0; arg = sep + 1; }

  if (!strcmp(line, "SERVO")) {

    servoManual = constrain(atoi(arg), 0, 180); mode = MANUAL; ack("SERVO", true);

  } else if (!strcmp(line, "RGB")) {

    int r, g, b;

    if (sscanf(arg, "%d,%d,%d", &r, &g, &b) == 3) {

      rgbManual[0] = constrain(r, 0, 255); rgbManual[1] = constrain(g, 0, 255); rgbManual[2] = constrain(b, 0, 255);

      mode = MANUAL; ack("RGB", true);

    } else ack("RGB", false);

  } else if (!strcmp(line, "BUZZ")) {

    int f, d;

    if (sscanf(arg, "%d,%d", &f, &d) == 2 && f >= 100 && f <= 5000 && d > 0 && d <= 2000) {

      tone(PIN_BUZZER, f, d); ack("BUZZ", true);

    } else ack("BUZZ", false);

  } else if (!strcmp(line, "MODE")) {

    if (arg[0] == 'A') { mode = AUTO; ack("MODE", true); }

    else if (arg[0] == 'M') { mode = MANUAL; ack("MODE", true); }

    else ack("MODE", false);

  } else if (!strcmp(line, "LIGHT")) {

    lightOn = (arg[0] == '1'); ack("LIGHT", true);

  } else if (!strcmp(line, "PING")) {

    ack("PING", true);

  } else {

    ack("UNKNOWN", false);

  }

}

void readSerial() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      if (rxLen) { rxBuf[rxLen] = 0; handleCommand(rxBuf); rxLen = 0; }

    } else if (rxLen < sizeof(rxBuf) - 1) {

      rxBuf[rxLen++] = c;

    } else {

      rxLen = 0;                                 // linha longa demais: descarta

    }

  }

}

// ---------------------------------------------------------------- setup / loop

void setup() {

  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);

  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_RGB_R, OUTPUT); pinMode(PIN_RGB_G, OUTPUT); pinMode(PIN_RGB_B, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), onEcho, CHANGE);

  Wire.begin();

  Wire.setClock(400000);                         // I2C Fast Mode (400 kHz)

  imuOk = imuInit();

  lcd.init();

  lcd.backlight();

  lcd.setCursor(0, 0); lcd.print(F("AssisTilt  2026.2"));

  lcd.setCursor(0, 1); lcd.print(imuOk ? F("MPU6050 OK") : F("MPU6050 FALHOU"));

  servo.attach(PIN_SERVO);

  servo.write(90);

  for (uint8_t i = 0; i < N_AVG; i++) {          // pré-carrega o filtro (sem "rampa" inicial)

    ldrBuf[i] = analogRead(PIN_LDR);

    ldrSum += ldrBuf[i];

  }

  lastImuUs = micros();

  Serial.println(F("{\"event\":\"boot\",\"fw\":\"assistilt-1.0\"}"));

}

void loop() {

  unsigned long now = millis();

  static unsigned long tLdr = 0, tImu = 0, tTelem = 0, tLcd = 0;

  readSerial();                                  // sempre ouvindo o App

  serviceUltrasonic(now);

  if (now - tLdr >= T_LDR) { tLdr = now; sampleLdr(); }

  if (now - tImu >= T_IMU) { tImu = now; updateAttitude(); }

  serviceGesture(now);

  serviceAlert(now);

  serviceActuators(now);

  if (now - tLcd >= T_LCD)     { tLcd = now;   serviceLcd(); }

  if (now - tTelem >= T_TELEM) { tTelem = now; sendTelemetry(now); }

}
