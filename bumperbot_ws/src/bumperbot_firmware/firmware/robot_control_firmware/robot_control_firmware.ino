// --- L298N Motor Driver Pins ---
#define L298N_enA 9    // Right motor PWM
#define L298N_enB 11   // Left motor PWM
#define L298N_in1 12   // Right motor dir 1
#define L298N_in2 13   // Right motor dir 2
#define L298N_in3 7    // Left motor dir 1
#define L298N_in4 8    // Left motor dir 2

// --- Encoder pins (all PORTD, one pin-change ISR) ---
// Right: A = D2, B = D4    Left: A = D3, B = D5
volatile long rc = 0, lc = 0;
volatile uint8_t prevR = 0, prevL = 0;
const int8_t T[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

ISR(PCINT2_vect) {
  uint8_t p = PIND;
  uint8_t r = (((p >> 2) & 1) << 1) | ((p >> 4) & 1);
  uint8_t l = (((p >> 3) & 1) << 1) | ((p >> 5) & 1);
  rc -= T[(prevR << 2) | r];   // right: forward = positive
  lc += T[(prevL << 2) | l];   // left:  forward = positive
  prevR = r;
  prevL = l;
}

// --- Velocity settings ---
const float CPR = 1320.0;                    // counts per wheel revolution
const float RAD_PER_COUNT = 2.0 * PI / CPR;
const unsigned long CTRL_DT_MS = 20;         // 50 Hz velocity + PID loop
const float ALPHA = 0.3;                     // velocity low-pass

// --- PID settings ---
const float RAMP = 50.0;                     // setpoint slew limit, rad/s^2
// Feedforward: PWM = offset + k * |speed|   (fitted from your 1..15 rad/s runs)
const float PWM0_R = 23.0, K_R = 3.5;
const float PWM0_L = 24.0, K_L = 3.9;
const float Kp = 4.0, Ki = 25.0;
const float I_LIMIT = 40.0;                  // max integral contribution (PWM units)

// --- Serial command settings ---
const float MAX_RAD = 20.0;                  // clamp on commanded speed, rad/s
const float MIN_RAD = 1.0;                   // NEW: non-zero commands below this are treated as 0
const unsigned long CMD_TIMEOUT_MS = 500;    // stop if no command for this long

long prevRc = 0, prevLc = 0;
float rVel = 0, lVel = 0;                    // measured, rad/s
float rTarget = 0, lTarget = 0;              // requested, rad/s (from serial)
float rRamp = 0, lRamp = 0;                  // ramped setpoint the PID chases
float rInt = 0, lInt = 0;                    // integrator states
int cmdR = 0, cmdL = 0;                      // PWM actually applied
unsigned long lastCtrl = 0, lastPrint = 0, lastCmd = 0;

char buf[32];
uint8_t bufLen = 0;

// --- Motor control ---
void setMotors(int rightSpeed, int leftSpeed) {
  cmdR = rightSpeed;
  cmdL = leftSpeed;

  if (rightSpeed > 0)      { digitalWrite(L298N_in1, HIGH); digitalWrite(L298N_in2, LOW);  }
  else if (rightSpeed < 0) { digitalWrite(L298N_in1, LOW);  digitalWrite(L298N_in2, HIGH); }
  else                     { digitalWrite(L298N_in1, LOW);  digitalWrite(L298N_in2, LOW);  }
  analogWrite(L298N_enA, abs(rightSpeed));

  if (leftSpeed > 0)       { digitalWrite(L298N_in3, HIGH); digitalWrite(L298N_in4, LOW);  }
  else if (leftSpeed < 0)  { digitalWrite(L298N_in3, LOW);  digitalWrite(L298N_in4, HIGH); }
  else                     { digitalWrite(L298N_in3, LOW);  digitalWrite(L298N_in4, LOW);  }
  analogWrite(L298N_enB, abs(leftSpeed));
}

// Move 'cur' toward 'goal' by at most RAMP*dt
float slew(float cur, float goal, float dt) {
  float maxStep = RAMP * dt;
  if (goal > cur + maxStep) return cur + maxStep;
  if (goal < cur - maxStep) return cur - maxStep;
  return goal;
}

// Feedforward (with deadband offset) + PI. The integrator only runs once the ramp
// has reached the final target.
int pidStep(float ramped, float finalTarget, float vel,
            float pwm0, float k, float &integ, float dt) {
  if (ramped == 0.0) { integ = 0; return 0; }          // stopped: coast, reset integrator
  float err = ramped - vel;
  if (ramped == finalTarget) {
    integ += Ki * err * dt;
    integ = constrain(integ, -I_LIMIT, I_LIMIT);
  }
  float sign = (ramped > 0) ? 1.0 : -1.0;
  float ff = sign * (pwm0 + k * fabs(ramped));         // CHANGED: offset + slope
  float out = ff + Kp * err + integ;
  if (ramped > 0 && out < 0) out = 0;                  // never drive against the target
  if (ramped < 0 && out > 0) out = 0;
  return (int)constrain(out, -255, 255);
}

// Parse one complete line: "V <right> <left>"  (rad/s)
void handleLine(char *line) {
  if (line[0] != 'V') return;
  char *e1; float r = strtod(line + 1, &e1);
  if (e1 == line + 1) return;                          // no number found
  char *e2; float l = strtod(e1, &e2);
  if (e2 == e1) return;                                // second number missing
  if (fabs(r) < MIN_RAD) r = 0;                        // NEW: below the stall limit -> stop
  if (fabs(l) < MIN_RAD) l = 0;
  rTarget = constrain(r, -MAX_RAD, MAX_RAD);
  lTarget = constrain(l, -MAX_RAD, MAX_RAD);
  lastCmd = millis();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) { buf[bufLen] = 0; handleLine(buf); }
      bufLen = 0;
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    } else {
      bufLen = 0;                                      // overflow: drop the line
    }
  }
}

void setup() {
  pinMode(L298N_enA, OUTPUT); pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT); pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT); pinMode(L298N_in4, OUTPUT);
  setMotors(0, 0);

  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP); pinMode(5, INPUT_PULLUP);

  uint8_t p = PIND;
  prevR = (((p >> 2) & 1) << 1) | ((p >> 4) & 1);
  prevL = (((p >> 3) & 1) << 1) | ((p >> 5) & 1);

  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21);

  lastCtrl = millis();
  lastCmd = millis();
}

void loop() {
  readSerial();                        // read commands first
  unsigned long now = millis();        // then take the time, so lastCmd <= now

  // Watchdog: no recent command -> stop
  if (now - lastCmd > CMD_TIMEOUT_MS) { rTarget = 0; lTarget = 0; }

  // Velocity + ramp + PID at 50 Hz
  if (now - lastCtrl >= CTRL_DT_MS) {
    float dt = (now - lastCtrl) / 1000.0;
    lastCtrl = now;

    noInterrupts(); long r = rc, l = lc; interrupts();
    float rRaw = (r - prevRc) * RAD_PER_COUNT / dt;
    float lRaw = (l - prevLc) * RAD_PER_COUNT / dt;
    prevRc = r; prevLc = l;
    rVel += ALPHA * (rRaw - rVel);
    lVel += ALPHA * (lRaw - lVel);

    rRamp = slew(rRamp, rTarget, dt);
    lRamp = slew(lRamp, lTarget, dt);

    setMotors(pidStep(rRamp, rTarget, rVel, PWM0_R, K_R, rInt, dt),
              pidStep(lRamp, lTarget, lVel, PWM0_L, K_L, lInt, dt));
  }

  // Telemetry at 20 Hz
  if (now - lastPrint >= 50) {
    lastPrint = now;
    noInterrupts(); long r = rc, l = lc; interrupts();
    Serial.print("R_vel:"); Serial.print(rVel, 2);
    Serial.print(",L_vel:"); Serial.print(lVel, 2);
    Serial.print(",R_cnt:"); Serial.print(r);
    Serial.print(",L_cnt:"); Serial.println(l);
  }
}