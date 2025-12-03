// Improved version with separated PWM/direction outputs and time-compensated PID

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// =========================================================
// 1. PIN DEFINITIES
// =========================================================

const int LED_PIN = 2;
const int BUTTON_PIN = 4;

// Motor pinnen (PWM + DIR)
const int LEFT_PWM = 10;
const int LEFT_DIR = 11;
const int RIGHT_PWM = 5;
const int RIGHT_DIR = 6;

// Sensoren
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint8_t SensorCount = 8;

// =========================================================
// 2. VARIABELEN & PARAMETERS
// =========================================================

struct param_t {
  unsigned long cycleTime; 
  int black[8];            
  int white[8];            
  int power;               
  float kp;
  float ki;
  float kd;
} params;

// Globale variabelen
int normalised[8];    
float iTerm = 0;      
float lastErr = 0;    
unsigned long previousMicros = 0;

bool isRunning = false;
bool requestCalib = false;

// Bluetooth Buffer
char cmdBuf[32];
uint8_t cmdLen = 0;

// Knop timing
unsigned long buttonPressStart = 0;
const long LONG_PRESS_MS = 1500;

// =========================================================
// 3. FUNCTIES: MOTOREN & KALIBRATIE
// =========================================================

void stopMotors() {
  // Originele motorstopping: beide PWM op 0 / LOW
  analogWrite(LEFT_PWM, 0);
  digitalWrite(LEFT_DIR, LOW);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(RIGHT_DIR, LOW);
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Originele gedrag (zoals in je eerste versie): PWM op één pin,
  // de andere pin LOW voor die richting.
  // LEFT
  if (leftSpeed >= 0) {
    analogWrite(LEFT_PWM, leftSpeed);
    digitalWrite(LEFT_DIR, LOW);
  } else {
    digitalWrite(LEFT_PWM, LOW);
    analogWrite(LEFT_DIR, -leftSpeed);
  }

  // RIGHT
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_PWM, rightSpeed);
    digitalWrite(RIGHT_DIR, LOW);
  } else {
    digitalWrite(RIGHT_PWM, LOW);
    analogWrite(RIGHT_DIR, -rightSpeed);
  }
}

void autoCalibrate(unsigned long ms) {
  stopMotors(); 
  Serial.println("Start Handmatige Kalibratie...");
  Serial.println("Beweeg de robot nu over ZWART en WIT...");

  for (int i = 0; i < SensorCount; i++) {
    params.black[i] = 0;    
    params.white[i] = 1023; 
  }

  unsigned long t0 = millis();
  
  while (millis() - t0 < ms) {
    if ((millis() / 100) % 2 == 0) digitalWrite(LED_PIN, HIGH);
    else digitalWrite(LED_PIN, LOW);

    for (int i = 0; i < SensorCount; i++) {
      int raw = analogRead(sensorPins[i]);
      if (raw > params.black[i]) params.black[i] = raw;
      if (raw < params.white[i]) params.white[i] = raw;
    }
  }

  digitalWrite(LED_PIN, LOW); 
  Serial.println("Kalibratie Klaar.");
}

// =========================================================
// 4. SETUP
// =========================================================

void setDefaultParams() {
  params.cycleTime = 2500; 
  params.power = 70;      
  params.kp = 5.0;
  params.ki = 0.0;
  params.kd = 0.0;

  for (int i = 0; i < 8; i++) {
    params.black[i] = 800; 
    params.white[i] = 100;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  setDefaultParams();
  stopMotors();
  Serial.println("Ready. Stuur 'kal' om te kalibreren of 'go' om te starten.");
}

// =========================================================
// 5. COMMANDO'S
// =========================================================

void handleCommand(const char* cmd) {
  if (strcmp(cmd, "kal") == 0) {
    requestCalib = true; 
    isRunning = false;
  }
  else if (strcmp(cmd, "go") == 0) isRunning = true;
  else if (strcmp(cmd, "stop") == 0) { isRunning = false; stopMotors(); }
  else if (strncmp(cmd, "kp=", 3) == 0) params.kp = atof(cmd+3);
  else if (strncmp(cmd, "ki=", 3) == 0) params.ki = atof(cmd+3);
  else if (strncmp(cmd, "kd=", 3) == 0) params.kd = atof(cmd+3);
  else if (strncmp(cmd, "bs=", 3) == 0) params.power = constrain(atoi(cmd+3), 0, 255);
}

void pollInputs() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }

  int state = digitalRead(BUTTON_PIN);
  if (state == LOW) {
    if (buttonPressStart == 0) buttonPressStart = millis();
    if (millis() - buttonPressStart > LONG_PRESS_MS && isRunning) {
      isRunning = false; stopMotors();
    }
  } else {
    if (buttonPressStart > 0) {
      long duration = millis() - buttonPressStart;
      if (duration > 50 && duration < LONG_PRESS_MS) {
        if (!isRunning) isRunning = true;
        else { isRunning = false; stopMotors(); }
      }
      buttonPressStart = 0;
    }
  }
}

// =========================================================
// 6. MAIN LOOP met tijd-gecompenseerde PID
// =========================================================

void loop() {
  pollInputs();

  if (requestCalib) {
    autoCalibrate(8000);
    requestCalib = false;
  }

  if (!isRunning) {
    stopMotors();
    iTerm = 0;
    lastErr = 0;
    return;
  }

  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= params.cycleTime) {
    unsigned long dtMicros = currentMicros - previousMicros;
    previousMicros = currentMicros;

    float dt = dtMicros / 1e6;

    // A. Sensoren lezen en normaliseren
    for (int i = 0; i < SensorCount; i++) {
      int raw = analogRead(sensorPins[i]);
      int val = map(raw, params.white[i], params.black[i], 1000, 0);
      normalised[i] = constrain(val, 0, 1000);
    }

    // B. Positie
    float position = 0;
    int index = 0;
    
    for (int i = 1; i < 8; i++) {
      if (normalised[i] < normalised[index]) index = i;
    }

    if (index == 0) position = -30;
    else if (index == 7) position = 30;
    else {
      int s_nul = normalised[index];
      int s_min = normalised[index-1];
      int s_plus = normalised[index+1];

      float b = (float)(s_plus - s_min) / 2.0;
      float a = (float)s_plus - b - s_nul;

      if (abs(a) < 0.1) position = index;
      else {
        position = -b / (2 * a);
        position += index;
      }

      position -= 3.5;
      position *= 9.525;
    }

    // C. PID met tijdscompensatie
    float error = -position;

    float P = params.kp * error;
    
    iTerm += params.ki * error * dt;
    iTerm = constrain(iTerm, -510, 510);

    float D = params.kd * ((error - lastErr) / dt);

    float output = P + iTerm + D;
    output = constrain(output, -510, 510);

    lastErr = error;

    int base = params.power;
    int leftSpeed = base + output;
    int rightSpeed = base - output;

    setMotors(leftSpeed, rightSpeed);
  }
}
