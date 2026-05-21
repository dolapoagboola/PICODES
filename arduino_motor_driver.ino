/*
 ================================================================
  Roomba Gas Detection Robot — Arduino Mega Motor Driver
 ================================================================
  Receives serial commands from Raspberry Pi 5 PID controller.

  Protocol (Serial0, 115200 baud):
    Incoming: "L<int>,R<int>\n"
      e.g.    "L180,R200\n"
    Outgoing: "OK:<left>,<right>\n"  (acknowledgement)

  Motor Driver: L298N (or similar H-bridge)
  Wiring:
    Left  Motor → ENA=Pin2(PWM), IN1=Pin22, IN2=Pin23
    Right Motor → ENB=Pin3(PWM), IN3=Pin24, IN4=Pin25

  Safety:
    - Watchdog: motors stop if no command received in 500ms
    - PWM clamped to 0–255
 ================================================================
*/

// ─── PIN DEFINITIONS ─────────────────────────────────────────
// Left Motor (L298N Channel A)
const int ENA  = 2;   // PWM speed
const int IN1  = 22;  // Direction
const int IN2  = 23;

// Right Motor (L298N Channel B)
const int ENB  = 3;   // PWM speed
const int IN3  = 24;
const int IN4  = 25;

// Optional status LED
const int LED_PIN = 13;

// ─── CONSTANTS ───────────────────────────────────────────────
const int    BAUD_RATE       = 115200;
const int    MAX_PWM         = 230;
const int    MIN_PWM         = 60;    // below this motors don't move
const unsigned long WATCHDOG_MS = 500; // stop if no command for 500ms

// ─── STATE ───────────────────────────────────────────────────
int  leftPWM  = 0;
int  rightPWM = 0;
unsigned long lastCmdTime = 0;
bool motorRunning = false;

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);   // To Raspberry Pi

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  stopMotors();

  Serial.println("READY");
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {

  // 1. Parse incoming serial command
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (parseCommand(cmd)) {
      lastCmdTime  = millis();
      motorRunning = true;
      driveMotors(leftPWM, rightPWM);
      // Acknowledge back to Pi
      Serial.print("OK:");
      Serial.print(leftPWM);
      Serial.print(",");
      Serial.println(rightPWM);
    } else if (cmd == "STOP") {
      stopMotors();
      motorRunning = false;
      Serial.println("STOPPED");
    }
  }

  // 2. Watchdog — stop if Pi goes silent
  if (motorRunning && (millis() - lastCmdTime > WATCHDOG_MS)) {
    stopMotors();
    motorRunning = false;
    Serial.println("WATCHDOG:TIMEOUT");
  }
}

// ─── PARSE COMMAND ───────────────────────────────────────────
/*
  Expects: "L<int>,R<int>"
  e.g.     "L180,R200"
  Returns true on success.
*/
bool parseCommand(String cmd) {
  if (cmd.length() < 5) return false;
  if (cmd.charAt(0) != 'L') return false;

  int commaIdx = cmd.indexOf(',');
  if (commaIdx < 2) return false;

  String leftStr  = cmd.substring(1, commaIdx);
  String rightStr = cmd.substring(commaIdx + 2); // skip 'R'

  if (rightStr.length() == 0 || leftStr.length() == 0) return false;

  int l = leftStr.toInt();
  int r = rightStr.toInt();

  // Clamp
  leftPWM  = constrain(l, 0, MAX_PWM);
  rightPWM = constrain(r, 0, MAX_PWM);

  return true;
}

// ─── DRIVE MOTORS ────────────────────────────────────────────
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor — forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  // Right motor — forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);

  // LED blink while running
  digitalWrite(LED_PIN, (millis() / 200) % 2);
}

// ─── STOP ────────────────────────────────────────────────────
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(LED_PIN, LOW);
}
