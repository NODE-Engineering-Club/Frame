// Simple Arduino sketch for LED control via serial
// Expected serial input (single line, terminated by '\n'):
//  - "R", "G", "B"  => light the corresponding LED
//  - combinations like "RG", "GB", "RGB" => multiple LEDs
//  - "0" => turn all LEDs off

const int RED_PIN = 11;   // PWM-capable pin
const int GREEN_PIN = 10; // PWM-capable pin
const int BLUE_PIN = 9;   // PWM-capable pin

void setup() {
  Serial.begin(115200);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

void setFromString(String s) {
  s.trim();
  if (s.length() == 0) {
    return;
  }

  // turn off everything first
  bool r = false;
  bool g = false;
  bool b = false;

  if (s == "0") {
    // explicitly switch all off
  } else {
    if (s.indexOf('R') >= 0) r = true;
    if (s.indexOf('G') >= 0) g = true;
    if (s.indexOf('B') >= 0) b = true;
  }

  digitalWrite(RED_PIN, r ? HIGH : LOW);
  digitalWrite(GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(BLUE_PIN, b ? HIGH : LOW);

  // echo for debug
  Serial.print("LEDs -> R:");
  Serial.print(r);
  Serial.print(" G:");
  Serial.print(g);
  Serial.print(" B:");
  Serial.println(b);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    setFromString(line);
  }
  // small delay to yield CPU
  delay(5);
}
