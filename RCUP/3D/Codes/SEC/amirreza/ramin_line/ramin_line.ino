/////////////////////////////////////////////////////////////////////////////
//  GOALKEEPER
//
//  Rebuilt from base_darvazeban to kill the two faults that code has:
//
//  1. IT COULD LOCK UP. Every state there was a bare while() -- while(KB <
//     ldr_sens), while(brorast != 2) -- so a sensor that never fires traps the
//     robot forever. Here nothing loops: one state runs one cycle of loop(),
//     and every state carries a timeout that forces it back to RETURN. There
//     is no path that can hold the robot still.
//
//  2. IT WANDERED OUT OF THE GOAL MOUTH. Its sweep had no limit at all. Here
//     the sideways command is cut the moment a side sharp says a wall is
//     close, so the keeper cannot walk itself out of the area it defends.
//
//  Position comes from the SHARPS, not from the line sensors. Sharps give a
//  continuous distance, so the keeper always knows roughly where it is; the
//  LDRs are binary and were the thing that kept stalling the old logic, so
//  they are used only as limits and as out detection.
//
//  States: LINE holds the penalty line and slides with the ball. CHASE goes
//  for a loose ball and tries to score. WAIT holds still rather than shoving
//  the ball over a side out. BACK peels off the front out. RETURN brings the
//  robot home from anywhere and is the fallback every timeout leads to.
//
//  Heading is held by a PID (idea from RoBorregos' RCJ keeper); a ball behind
//  the robot is approached off-axis, never straight back (from Soccer-Robots'
//  DIFENSORE), so the keeper never pushes it into its own goal.
//
//  Switch on over GREEN -- setup() takes the LDR baselines wherever it starts.
/////////////////////////////////////////////////////////////////////////////
#include <TDAxis12.h>
#include <Wire.h>
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);
#define MAX_MOTOR 35000

//// speeds
#define V_SWEEP 27000  // sliding along the penalty line
#define V_HOME 27000   // running back to the line
#define V_CHASE 30000  // going for a loose ball
#define V_BACK 30000   // peeling off the front out

//// sharp thresholds -- CALIBRATE THESE ON THE PITCH.
//  Larger reading = wall is closer. Park the robot by hand on the penalty
//  line and read SHB off the OLED; that value is SHB_TARGET.
#define SHB_TARGET 1300  // back-sharp reading while sitting on the line
#define SHB_BAND 200     // half-width of the depth band counted as "in place"
#define SIDE_WALL 1300   // side-sharp reading that means a wall is right there
#define DEPTH_GAIN 90    // how hard the depth error pulls the robot back

//// ball geometry, TSOP frame: 0 ahead, +90 right, 180 behind
#define BALL_NEAR 6000        // balldistance close enough to leave the line for
#define BALL_SIDE 80          // out to this the ball is worth shading across for
#define BALL_BEHIND_ANGLE 150 // peel-off angle for a ball behind the keeper
#define BALL_SHIFT 0.6        // curve added when chasing, to get behind the ball

//// timeouts, in loop cycles. Nothing in this sketch may run unbounded.
#define T_CHASE 1500
#define T_WAIT 400
#define T_BACK 250
#define T_RETURN 3000

#define KICK_HOLD 20  // cycles the solenoid stays energised
#define KICK_COOL 60  // cycles it must rest afterwards

//// heading PID. GYRO_KP is the old plain gain, so KI = KD = 0 reproduces the
//  original proportional behaviour exactly. If it misbehaves: zero KI first,
//  then lower KD.
#define GYRO_KP 480
#define GYRO_KI 0.4
#define GYRO_KD 1200
#define GYRO_I_LIMIT 5000

#define GK_LINE 0
#define GK_CHASE 1
#define GK_WAIT 2
#define GK_BACK 3
#define GK_RETURN 4

float ballangle;
float balldistance;
float sensorangle;
float gy = 0, dgy = 0;
float value;
int shb, shl, shr, d;
int kf_s, kr_s, kb_s, kl_s;
int ldr_sens = 300;
int start_time = 0;
bool kf, kr, kb, kl;
bool isball;
bool ball_in_kicker = false;
bool OLED_EN = true;

// The robot is switched on somewhere green, so it always starts by driving
// home rather than assuming it is already in place.
int gk_state = GK_RETURN;
int state_cnt = 0;   // cycles spent in the current state; drives every timeout
int wait_side = 0;   // +1 waiting on the right out, -1 on the left
int kick_timer = 0;  // >0 firing, <0 cooling down

// Heading PID state. The correction is worked out once per loop and every
// motor() call in that loop reuses it, so a state that hands off to another
// cannot double-count the integral or flatten the derivative.
float heading_correction = 0;
float gyro_err_prev = 0;
float gyro_integral = 0;

void motor(int ml1, int ml2, int mr2, int mr1) {
  ml1 += heading_correction;
  ml2 += heading_correction;
  mr2 += heading_correction;
  mr1 += heading_correction;
  int maxVal = abs(ml1);

  if (abs(ml2) > maxVal) maxVal = abs(ml2);
  if (abs(mr2) > maxVal) maxVal = abs(mr2);
  if (abs(mr1) > maxVal) maxVal = abs(mr1);
  if (maxVal > MAX_MOTOR) {
    float scale = (float)MAX_MOTOR / maxVal;
    ml1 *= scale;
    ml2 *= scale;
    mr2 *= scale;
    mr1 *= scale;
  }
  if (ml1 > 65535) ml1 = 65535;
  if (ml2 > 65535) ml2 = 65535;
  if (mr2 > 65535) mr2 = 65535;
  if (mr1 > 65535) mr1 = 65535;

  if (ml1 < -65535) ml1 = -65535;
  if (ml2 < -65535) ml2 = -65535;
  if (mr2 < -65535) mr2 = -65535;
  if (mr1 < -65535) mr1 = -65535;
  /////////////////  MR1
  if (mr1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, mr1);
  } else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, mr1 + 65535);
  }
  /////////////////  MR2
  if (mr2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, mr2);
  } else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, mr2 + 65535);
  }
  /////////////////  ML1
  if (ml1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PB9, ml1);
  } else {
    digitalWrite(PB15, 1);
    pwmWrite(PB9, ml1 + 65535);
  }
  /////////////////  ML2
  if (ml2 > 0) {
    digitalWrite(PB14, 0);
    pwmWrite(PB8, ml2);
  } else {
    digitalWrite(PB14, 1);
    pwmWrite(PB8, ml2 + 65535);
  }
}
void movexy(float vx, float vy) {
  motor(vy + vx, vy - vx, -vy - vx, -vy + vx);
}
void moveSpeed(float angle, float s) {
  float vx = sin(radians(angle)) * s;
  float vy = cos(radians(angle)) * s;
  movexy(vx, vy);
}
void stop() {
  motor(0, 0, 0, 0);
}
float clamp(float val, float min_val, float max_val) {
  if (val > max_val) return max_val;
  if (val < min_val) return min_val;
  return val;
}
int pos(int val) {
  if (val < 0) return 0;
  else return val;
}
bool BOOL(float val, float shart) {
  if (val > shart) return true;
  else return false;
}
void sensor() {
  kf = BOOL(pos(analogRead(PA5) - kf_s), ldr_sens);
  kr = BOOL(pos(analogRead(PA6) - kr_s), ldr_sens);
  kb = BOOL(pos(analogRead(PA7) - kb_s), ldr_sens);
  kl = BOOL(pos(analogRead(PB0) - kl_s), ldr_sens);
  shb = analogRead(PA1);
  shr = analogRead(PA2);
  shl = analogRead(PA3);
  d = (shl - shr) * 30;
  ////////////////////////////// TSOP angle
  float sumx = 0, sumy = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8, (i / 1) % 2);
    digitalWrite(PB1, (i / 2) % 2);
    digitalWrite(PC14, (i / 4) % 2);
    digitalWrite(PC15, (i / 8) % 2);
    sensorangle = radians(i * 22.5);
    value = 4095 - analogRead(PA0);
    sumx += value * cos(sensorangle);
    sumy += value * sin(sensorangle);
  }
  ballangle = degrees(atan2(sumy, sumx));
  balldistance = sqrt(pow(sumx, 2) + pow(sumy, 2));
  if (balldistance > 2000) isball = true;
  else isball = false;
  if (!digitalRead(PA4) && balldistance > 7500 && (-90 < ballangle && ballangle < 90)) ball_in_kicker = true;
  else ball_in_kicker = false;

  gy = gyro.read();
  if (millis() - start_time > 5000) {
    OLED_EN = false;
    display.clearDisplay();
    display.display();
  }
}
void printall() {
  if (!OLED_EN) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("BA:");
  display.println(ballangle);
  display.print("BD:");
  display.println(balldistance);
  display.print("GY:");
  display.println(gy);
  display.print("SHR:");
  display.println(shr);
  display.print("SHL:");
  display.println(shl);
  display.print("SHB:");
  display.println(shb);
  display.print("ST:");
  display.println(gk_state);
  if (kf) display.fillRoundRect(74, 0, 40, 3, 0, WHITE);
  else display.drawRoundRect(74, 0, 40, 3, 0, WHITE);
  if (kr) display.fillRoundRect(123, 12, 3, 40, 0, WHITE);
  else display.drawRoundRect(123, 12, 3, 40, 0, WHITE);
  if (kb) display.fillRoundRect(74, 61, 40, 3, 0, WHITE);
  else display.drawRoundRect(74, 61, 40, 3, 0, WHITE);
  if (kl) display.fillRoundRect(62, 12, 3, 40, 0, WHITE);
  else display.drawRoundRect(62, 12, 3, 40, 0, WHITE);
  display.drawCircle(94, 32, 20, WHITE);
  display.fillTriangle(
    94 + 15 * sin(radians(gy)), 32 - 15 * cos(radians(gy)),
    94 + 22.5 * sin(radians(gy + 21)), 32 - 22.5 * cos(radians(gy + 21)),
    94 + 22.5 * sin(radians(gy - 21)), 32 - 22.5 * cos(radians(gy - 21)), BLACK);
  display.drawLine(
    94 + 20 * sin(radians(gy + 20)), 32 - 20 * cos(radians(gy + 20)),
    94 + 12 * sin(radians(gy + 30)), 32 - 12 * cos(radians(gy + 30)), WHITE);
  display.drawLine(
    94 + 20 * sin(radians(gy - 20)), 32 - 20 * cos(radians(gy - 20)),
    94 + 12 * sin(radians(gy - 30)), 32 - 12 * cos(radians(gy - 30)), WHITE);
  display.drawLine(
    94 + 12 * sin(radians(gy + 30)), 32 - 12 * cos(radians(gy + 30)),
    94 + 12 * sin(radians(gy - 30)), 32 - 12 * cos(radians(gy - 30)), WHITE);
  if (ball_in_kicker) display.fillCircle(93 + sin(radians(gy)) * 17, 32 - cos(radians(gy)) * 17, 2, WHITE);
  else if (isball) display.fillCircle(93 + sin(radians(ballangle)) * 25, 32 - cos(radians(ballangle)) * 25, 2, WHITE);
  display.display();
}

// Every state change goes through here, so the timeout counter can never be
// left stale from the state before.
void go(int s) {
  gk_state = s;
  state_cnt = 0;
}

// Recomputes the heading correction for this cycle. The integral is clamped
// before it is scaled, so a long shove against the robot cannot wind it up
// into a spin that outlives the shove.
void update_heading_pid() {
  float error = clamp(gy - dgy, -90, 90);
  gyro_integral = clamp(gyro_integral + error, -GYRO_I_LIMIT, GYRO_I_LIMIT);
  heading_correction = error * GYRO_KP
                       + gyro_integral * GYRO_KI
                       + (error - gyro_err_prev) * GYRO_KD;
  gyro_err_prev = error;
}

// Fires the kicker when the ball is captured, then releases and rests. A
// counter rather than base_darvazeban's delay(200), so the robot keeps
// steering and reading sensors while the solenoid is energised.
void kicker() {
  if (kick_timer > 0) {
    kick_timer--;
    if (kick_timer == 0) {
      digitalWrite(PC13, 0);
      kick_timer = -KICK_COOL;
    }
  } else if (kick_timer < 0) {
    kick_timer++;
  } else if (ball_in_kicker) {
    digitalWrite(PC13, 1);
    kick_timer = KICK_HOLD;
  }
}

// The keeper is in place when the back sharp puts it inside the depth band.
// This is a continuous measure on purpose: the old code waited on a binary
// LDR that could simply never fire, and then it waited forever.
bool in_place() {
  return abs(shb - SHB_TARGET) < SHB_BAND;
}

// Forward/back push that pins the robot on the line. shb rises as the back
// wall nears, so sitting too deep pushes forward.
float depth_hold() {
  float push = clamp((shb - SHB_TARGET) * DEPTH_GAIN, -V_HOME, V_HOME);
  // Never drive across a line the robot is already standing on.
  if (push < 0 && kb) push = 0;
  if (push > 0 && kf) push = 0;
  return push;
}

// Sideways command, cut off at the walls. This is the limit base_darvazeban
// never had: whatever the ball does, the keeper will not slide itself out of
// the goal mouth.
float sweep_lateral() {
  float lateral = clamp(sin(radians(ballangle)) * 2.0, -1.0, 1.0) * V_SWEEP;
  if (!isball) lateral = 0;
  if (lateral > 0 && (shr > SIDE_WALL || kr)) lateral = 0;
  if (lateral < 0 && (shl > SIDE_WALL || kl)) lateral = 0;
  return lateral;
}

// Holding the penalty line: slide left and right with the ball while the
// depth term keeps the robot on it.
void gk_line() {
  if (!in_place()) {
    go(GK_RETURN);
    return;
  }
  // A loose ball close enough to be worth leaving the line for.
  if (isball && balldistance > BALL_NEAR) {
    go(GK_CHASE);
    return;
  }
  if (isball && (ballangle > BALL_SIDE || ballangle < -BALL_SIDE)) {
    // Behind the keeper: come round it rather than reversing into it.
    moveSpeed(ballangle > 0 ? BALL_BEHIND_ANGLE : -BALL_BEHIND_ANGLE, V_SWEEP);
    return;
  }
  movexy(sweep_lateral(), depth_hold());
}

// Going for a loose ball and trying to put it away. The curve added to the
// ball angle is what swings the robot behind the ball instead of nudging it
// sideways.
void gk_chase() {
  if (state_cnt > T_CHASE || !isball) {
    go(GK_RETURN);
    return;
  }
  // Ball driven onto a side out: stop rather than shove it over.
  if (kr && ballangle > 0) {
    wait_side = 1;
    go(GK_WAIT);
    return;
  }
  if (kl && ballangle < 0) {
    wait_side = -1;
    go(GK_WAIT);
    return;
  }
  // Over the front out with the ball: back off and go home.
  if (kf && ballangle > -90 && ballangle < 90) {
    go(GK_BACK);
    return;
  }
  kicker();
  if (ballangle > BALL_SIDE || ballangle < -BALL_SIDE) {
    moveSpeed(ballangle > 0 ? BALL_BEHIND_ANGLE : -BALL_BEHIND_ANGLE, V_CHASE);
  } else {
    float shift = clamp(ballangle * BALL_SHIFT, -45, 45);
    moveSpeed(ballangle + shift, V_CHASE);
  }
}

// Ball is sitting out past a side line. Hold still -- pushing on would only
// put it out -- but never hold longer than T_WAIT.
void gk_wait() {
  bool still_out = (wait_side > 0) ? (kr && ballangle > 0) : (kl && ballangle < 0);
  if (state_cnt > T_WAIT || !still_out || !isball) {
    go(GK_RETURN);
    return;
  }
  stop();
}

// Came over the front out. Reverse for a fixed count, then go home.
void gk_back() {
  if (state_cnt > T_BACK) {
    go(GK_RETURN);
    return;
  }
  movexy(0, -V_BACK);
}

// Home from anywhere. Depth from the back sharp, lateral from the sharp
// difference, and an out line under the robot is peeled off first. This state
// cannot stall: it steers on continuous sharp readings, and its own timeout
// simply restarts it.
void gk_return() {
  if (state_cnt > T_RETURN) {
    state_cnt = 0;  // nothing better to try; keep steering rather than freeze
  }
  if (in_place()) {
    go(GK_LINE);
    return;
  }
  // Peel off whichever out line is under the robot before anything else.
  if (kf) {
    movexy(0, -V_HOME);
    return;
  }
  if (kr) {
    movexy(-V_HOME, 0);
    return;
  }
  if (kl) {
    movexy(V_HOME, 0);
    return;
  }
  float push = clamp((shb - SHB_TARGET) * DEPTH_GAIN, -V_HOME, V_HOME);
  movexy(clamp(d, -V_HOME, V_HOME), push);
}

void setup() {
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
  pinMode(PA8, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, 0);
  delay(250);
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("loading00");
  display.display();
  i2c.begin();
  // Baselines are taken wherever the robot is switched on, so switch it on
  // over GREEN -- calibrating on white would make every line invisible.
  int kf_s_sum = 0;
  int kr_s_sum = 0;
  int kb_s_sum = 0;
  int kl_s_sum = 0;
  for (int i = 0; i < 10; i++) {
    kf_s_sum += analogRead(PA5);
    kr_s_sum += analogRead(PA6);
    kb_s_sum += analogRead(PA7);
    kl_s_sum += analogRead(PB0);
  }
  kf_s = kf_s_sum / 10;
  kr_s = kr_s_sum / 10;
  kb_s = kb_s_sum / 10;
  kl_s = kl_s_sum / 10;
}
void loop() {
  sensor();
  printall();
  // The keeper stays square to its own goal, so the striker's wall-aiming
  // swing of dgy is never applied here.
  dgy = 0;
  update_heading_pid();
  state_cnt++;

  switch (gk_state) {
    case GK_LINE: gk_line(); break;
    case GK_CHASE: gk_chase(); break;
    case GK_WAIT: gk_wait(); break;
    case GK_BACK: gk_back(); break;
    default: gk_return(); break;
  }
}
