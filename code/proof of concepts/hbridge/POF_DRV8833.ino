// Motor 1
const int IN1 = 5;   // PWM
const int IN2 = 4;   // PWM
 
// Motor 2
const int IN3 = 3;   // PWM
const int IN4 = 2;  // PWM
 
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
 
  // alles uit
  stopMotor1();
  stopMotor2();
}
 
void loop() {
 
  // ---------------- MOTOR 1 -----------------
 
  // Motor 1 vooruit langzaam
  motor1Forward(100);
  delay(2000);
 
  // Motor 1 vooruit snel
  motor1Forward(255);
  delay(2000);
 
  // Motor 1 achteruit langzaam
  motor1Backward(120);
  delay(2000);
 
  // Motor 1 uit
  stopMotor1();
  delay(2000);
 
 
  // ---------------- MOTOR 2 -----------------
 
  // Motor 2 vooruit langzaam
  motor2Forward(80);
  delay(2000);
 
  // Motor 2 vooruit snel
  motor2Forward(255);
  delay(2000);
 
  // Motor 2 achteruit met medium snelheid
  motor2Backward(150);
  delay(2000);
 
  // Motor 2 uit
  stopMotor2();
  delay(2000);
}
 
//
// ----------- FUNCTIES MOTOR 1 -----------
//
 
void motor1Forward(int speed) {
  analogWrite(IN1, speed);  // PWM
  analogWrite(IN2, 0);
}
 
void motor1Backward(int speed) {
  analogWrite(IN1, 0);
  analogWrite(IN2, speed);  // PWM
}
 
void stopMotor1() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}
 
//
// ----------- FUNCTIES MOTOR 2 -----------
//
 
void motor2Forward(int speed) {
  analogWrite(IN3, speed);
  analogWrite(IN4, 0);
}
 
void motor2Backward(int speed) {
  analogWrite(IN3, 0);
  analogWrite(IN4, speed);
}
 
void stopMotor2() {
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}