// WP4 ESP-side firmware: parse Sortify commands, echo back ACKs.
//
// Protocol (line-based, ASCII, '\n' terminated):
//   TURN +30      turn 30 deg clockwise (right)
//   TURN -45      turn 45 deg counter-clockwise (left)
//   MOVE 200      drive forward 200 ms
//   MOVE -100     drive backward 100 ms
//   GRIP O / C    gripper open / close
//   STOP          emergency stop
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

String buf = "";

void handleCommand(const String& cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "PING") {
    Serial.print("PONG\n");
    return;
  }

  if (cmd.startsWith("TURN ")) {
    int deg = cmd.substring(5).toInt();
    // TODO (WP5): drive motors to rotate `deg` degrees.
    // Sign: + = clockwise (right), - = counter-clockwise (left).
    Serial.print("ACK TURN ");
    Serial.print(deg);
    Serial.print("\n");
    return;
  }

  if (cmd.startsWith("MOVE ")) {
    int ms = cmd.substring(5).toInt();
    // TODO (WP5): run motors for `ms` ms. + = forward, - = backward.
    Serial.print("ACK MOVE ");
    Serial.print(ms);
    Serial.print("\n");
    return;
  }

  if (cmd.startsWith("GRIP ")) {
    char op = cmd.charAt(5);
    // TODO (WP5): drive the gripper servo. 'O' open, 'C' close.
    Serial.print("ACK GRIP ");
    Serial.print(op);
    Serial.print("\n");
    return;
  }

  if (cmd == "STOP") {
    // TODO (WP5): cut motor power immediately.
    Serial.print("ACK STOP\n");
    return;
  }

  Serial.print("ERR ");
  Serial.print(cmd);
  Serial.print("\n");
}

void setup() {
  Serial.begin(9600);
  delay(2000);
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
}
