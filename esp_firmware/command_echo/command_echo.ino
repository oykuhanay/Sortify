// WP4 ESP-side firmware: parse Sortify commands, echo back ACKs.
// AAA
// Protocol (line-based, ASCII, '\n' terminated):
//   TURN +030     turn 30 deg clockwise (right)   (signed 3-digit)
//   TURN -045     turn 45 deg counter-clockwise   (signed 3-digit)
//   MOVE 12.34    drive forward 12.34 cm          (unsigned 2.2 fixed)
//   GRIP O / C    gripper open / close
//   STOP          emergency stop
//   TRIM R 25.0   set right motor speed scaler
//   TRIM L 18.5   set left  motor speed scaler
//   PING          (legacy) replies PONG
//
// What this sketch does:
//   Parses commands as they arrive on the HM-10 UART and replies with
//   "ACK <verb> <arg>" so the PC side knows the robot received and
//   understood the command. Motor / servo actuation is WP5's job and
//   gets dropped in where the TODO comments are.
//
// Wiring + upload notes: see esp_firmware/pingpong/pingpong.ino header.
// In short: HM-10 RXD -> ESP TX (GPIO1), HM-10 TXD -> ESP RX (GPIO3).
// Unplug the HM-10 TX/RX jumpers before uploading.

#include <Servo.h>

// 0 d0 pini kıskaç için tanımlandı
#define PWMA 4   // Sağ motor hız
#define AIN1 12
#define AIN2 13

#define PWMB 14   // Sol motor hız
#define BIN1 15
#define BIN2 2

#define STBY 5

// ESP8266 analogWrite araligi: 0 - 1023
int baseSpeed = 675;

// Motor hiz duzeltme katsayilari

Servo sg90;

String buf = "";
// Per-motor trim, interpreted as a percentage of baseSpeed.
// Final PWM written to the motor = baseSpeed * trim / 100.
// Calibrated 2026-06-20: the left motor is meaningfully stronger than
// the right, so the left trim runs much lower to keep the robot tracking
// straight. Adjust live with TRIM commands; for slower motion drop both
// proportionally (e.g. R=60, L=16.5 keeps the same 0.275 ratio).
float r_m_speed = 80.00;  // A Motoru, 0-100 percent
float l_m_speed = 22.00;  // B Motoru, 0-100 percent

// Calibration constants: tune these by experiment.
// How many cm the robot travels in 1 second of forward motion at the
// current trim values, and how many degrees it rotates in 1 second.
// Measure once with a ruler / protractor then update.
float CM_PER_SEC  = 20.0;   // <-- TUNE
float DEG_PER_SEC = 90.0;   // <-- TUNE

void handleCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "PING") {
    Serial.print("PONG\n");
    return;
  }

  if (cmd.startsWith("TURN ")) {
    int deg = cmd.substring(5).toInt();   // "+030" -> 30, "-045" -> -45
    // Sign: + = clockwise (right), - = counter-clockwise (left).
    int ms = (int)((abs(deg) / DEG_PER_SEC) * 1000.0);
    if (deg >= 0) {
      turnRight(ms);
    } else {
      turnLeft(ms);
    }
    Serial.print("ACK TURN ");
    Serial.print(deg);
    Serial.print("\n");
    return;
  }

  if (cmd.startsWith("MOVE ")) {
    float cm = cmd.substring(5).toFloat();   // "05.00" -> 5.0, "12.34" -> 12.34
    // Forward only (no reverse this term). Convert cm to motor-on ms.

    int ms = (int)((cm / CM_PER_SEC) * 1000.0);

    if(cm>0){
      moveForward(ms);
    }
    else if(cm<0){
      moveBackward(abs(ms));
    }
    
    Serial.print("ACK MOVE ");
    Serial.print(cm);
    Serial.print("\n");
    return;
  }

  if (cmd.startsWith("TRIM ")) {
    char side = cmd.charAt(5);              // 'R' or 'L'
    float val = cmd.substring(7).toFloat(); // numeric value
    if (side == 'R') {
      r_m_speed = val;
    } else if (side == 'L') {
      l_m_speed = val;
    }
    Serial.print("ACK TRIM ");
    Serial.print(side);
    Serial.print(" ");
    Serial.print(val);
    Serial.print("\n");
    return;
  }

  if (cmd.startsWith("GRIP ")) {
    char op = cmd.charAt(5);
    if (op == 'O') {
      sg90.write(0);     // aç
    }
    else if (op == 'C') {
      sg90.write(150);    // kapa
    }
    // TODO (WP5): drive the gripper servo. 'O' open, 'C' close.
    //Serial.print("ACK GRIP ");
    //Serial.print(op);
    //Serial.print("\n");
    return;
  }

  if (cmd == "STOP") {
    stopMotors();
    Serial.print("ACK STOP\n");
    return;
  }

  Serial.print("ERR ");
  Serial.print(cmd);
  Serial.print("\n");
}

void setup() {
  // Configure all motor-related pins as OUTPUT *before* anything else,
  // and force them LOW. ESP8266 boot leaves GPIO15/GPIO2 (BIN1/BIN2)
  // floating for ~150 ms, which is enough for the motor driver to twitch
  // the wheels. Holding STBY LOW while the rest of setup runs keeps the
  // driver disabled until we are ready, so those boot transients never
  // reach the motors.
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);              // motor driver disabled first

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  sg90.attach(0);   //Servo sürmek için D0 pini tanımlanıyor...
  sg90.write(0);
  Serial.begin(9600);
  delay(2000);

  // Now everything is in a known idle state — safe to enable the driver.
  digitalWrite(STBY, HIGH);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(buf);
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 64) buf = "";  // discard pathological lines
    }
  }

  

  //testing the motors in forward direction
  //int pwm1 = r_m_speed*1023/100;
  //digitalWrite(AIN1, LOW);
  //digitalWrite(AIN2, HIGH);
  //analogWrite(PWMA, pwm1);

  //int pwm2 = l_m_speed*1023/100;
  //digitalWrite(BIN1, HIGH);
  //digitalWrite(BIN2, LOW);
  //analogWrite(PWMB, pwm2);

  //Remove later:
  //moveForward(1000);
  //delay(2000);
}


void setRightMotor(int speed)
{
  // `speed` is already a ready-to-write PWM value. Trims live in the
  // higher-level move* / turn* helpers; do NOT multiply by r_m_speed
  // here or we'd apply the trim twice and overflow the 0-1023 range.
  speed = constrain(speed, -1023, 1023);

  if (speed > 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -speed);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void setLeftMotor(int speed)
{
  // Same rule as setRightMotor: trim is already applied upstream.
  speed = constrain(speed, -1023, 1023);

  if (speed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

// Debug helper: dump the actual PWM values and trim state we are about
// to use, so we can see from the REPL whether TRIM is taking effect.
// Lines start with "DBG " so the REPL prints them as <-arrows like any
// other reply and we can spot them easily.
void dbgPrintMotion(const char* tag, int rightSpeed, int leftSpeed) {
  Serial.print("DBG ");
  Serial.print(tag);
  Serial.print(" R=");
  Serial.print(rightSpeed);
  Serial.print(" L=");
  Serial.print(leftSpeed);
  Serial.print(" trimR=");
  Serial.print(r_m_speed);
  Serial.print(" trimL=");
  Serial.print(l_m_speed);
  Serial.print("\n");
}

void moveForward(int durationMs)
{
  int rightSpeed = baseSpeed * r_m_speed/100;
  int leftSpeed  = baseSpeed * l_m_speed/100;

  dbgPrintMotion("FWD", rightSpeed, leftSpeed);

  setRightMotor(rightSpeed);
  setLeftMotor(leftSpeed);

  delay(durationMs);
  stopMotors();
}

void moveBackward(int durationMs)
{
  int rightSpeed = baseSpeed * r_m_speed/100;
  int leftSpeed  = baseSpeed * l_m_speed/100;

  dbgPrintMotion("BWD", -rightSpeed, -leftSpeed);

  setRightMotor(-rightSpeed);
  setLeftMotor(-leftSpeed);

  delay(durationMs);
  stopMotors();
}

void turnRight(int durationMs)
{
  int rightSpeed = baseSpeed * r_m_speed/100;
  int leftSpeed  = baseSpeed * l_m_speed/100;

  dbgPrintMotion("TR", -rightSpeed, leftSpeed);

  setRightMotor(-rightSpeed);
  setLeftMotor(leftSpeed);

  delay(durationMs);
  stopMotors();
}

void turnLeft(int durationMs)
{
  int rightSpeed = baseSpeed * r_m_speed/100;
  int leftSpeed  = baseSpeed * l_m_speed/100;

  dbgPrintMotion("TL", rightSpeed, -leftSpeed);

  setRightMotor(rightSpeed);
  setLeftMotor(-leftSpeed);

  delay(durationMs);
  stopMotors();
}

void stopMotors()
{
  // Cut PWM first so the driver stops sourcing current immediately,
  // then drive direction pins low to put both motor channels in brake/idle.
  // Forgetting to zero the PWM is what made the wheels keep spinning
  // forever after a MOVE — direction = LOW + PWM != 0 leaves the driver
  // in an undefined state on TB6612-style modules.
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}