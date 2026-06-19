// WP4 Link 3 ESP-side firmware: reply PONG to PING over HM-10 BLE-UART.
//
// Hardware:
//   ESP8266 NodeMCU V3 (32-pin Lolin variant) + HM-10 / BT05 module
//
// Wiring (see LEARNING_LOG.md Session 2 for full table):
//   HM-10 VCC  -> NodeMCU 3.3V
//   HM-10 GND  -> NodeMCU GND
//   HM-10 RXD  -> NodeMCU TX (GPIO1)   left rail, just under EN
//   HM-10 TXD  -> NodeMCU RX (GPIO3)   left rail, just under TX
//
// IMPORTANT — upload workflow:
//   GPIO1/GPIO3 are shared with the USB bootloader. Unplug HM-10's RXD
//   and TXD jumpers before clicking Upload, replug after upload finishes
//   ("Hash of data verified"). Leaving them connected causes:
//     "Failed to connect to ESP8266: Timed out waiting for packet header"
//
// Behaviour:
//   Reads bytes from Serial (= HM-10 UART). On every newline, if the
//   accumulated line contains "PING", replies "PONG" out the same UART.
//   Anything else is ignored. Buffer caps at 32 bytes to avoid runaway.

String buf = "";

void setup() {
  Serial.begin(9600);
  delay(2000);  // give HM-10 a moment to settle after power-on
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.indexOf("PING") >= 0) {
        Serial.print("PONG");
      }
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 32) buf = "";
    }
  }
}
