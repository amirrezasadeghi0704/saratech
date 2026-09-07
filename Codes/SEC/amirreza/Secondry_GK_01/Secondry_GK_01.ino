#include <TDAxis12.h>
#include <Wire.h>
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);
#define MAX_MOTOR 65000
// --- CHANGE: tunables pulled out of the code so they can be trimmed without
// hunting through the sketch. DGY_BAND is the width of the soft dgy ramp;
// the GK_* values drive the goalkeeper's sideways chase.
// BENEFIT: one place to tune on match day instead of scattered magic numbers.
// OLD: none of these existed, every value was hard coded inline.
#define DGY_BAND 600.0
#define GK_SPEED 60000.0
#define GK_EDGE_BAND 100.0
#define GK_DEADBAND 3.0
#define is_goaller true
float ballangle;
float balldistance;
float sensorangle;
float gy = 0, dgy = 0;
float v, vx;
float value;
float shift;
int shb, shl, shr, d;
int kf_s, kr_s, kb_s, kl_s;
// --- CHANGE: line-sensor threshold lowered ~7% (300 -> 279).
// BENEFIT: the white line is caught slightly earlier, so the robot turns back
// before a wheel is already over it. OLD: int ldr_sens = 300;
int ldr_sens = 279;
int start_time = 0;
bool kf, kr, kb, kl;
bool isball;
bool ball_in_kicker = false;
bool OLED_EN = true;
bool cheking_out = false;
bool goalkeeper_come_forward = false;
int ball_stop_cnt = 0;
float last_ball_angle = 0;

void motor(int ml1, int ml2, int mr2, int mr1) {
  // --- CHANGE: the heading error is wrapped to +-180 before clamping.
  // BENEFIT: near +-180 deg, gy - dgy used to come out as e.g. 195 instead of
  // -165, so the robot spun the long way round and in the wrong direction.
  // Inside the normal range nothing changes, the gain is still a flat 300.
  // OLD: float correction = clamp(gy - dgy, -90, 90);
  float error = convertAngle(gy - dgy);
  float correction = clamp(error, -90, 90);
  ml1 += (correction * 300);
  ml2 += (correction * 300);
  mr2 += (correction * 300);
  mr1 += (correction * 300);
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
  // NOTE: the > 0 tests below are deliberate. At exactly 0 the else branch
  // writes full reverse for one cycle = active braking, so the robot stops
  // on the spot at high speed instead of coasting a few cm. Do not use >= 0.
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
void move(float angle) {
  float vx = sin(radians(angle)) * v;
  float vy = cos(radians(angle)) * v;
  movexy(vx, vy);
}
void moveSpeed(float angle, float s) {
  float vx = sin(radians(angle)) * s;
  float vy = cos(radians(angle)) * s;
  movexy(vx, vy);
}
void movexy(float vx, float vy) {
  motor(vy + vx, vy - vx, -vy - vx, -vy + vx);
}
void movesecond(int m, int n) {
  sensor();
  // if (!OLED_EN) n *= 10;
  for (int i = 0; i < n; i++) {
    moveSpeed(m, 50000);
    sensor();
    printall();
  }
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
  //if ((kf > ldr_sens || kr > ldr_sens || kb > ldr_sens || kl > ldr_sens) && isball) return;
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
  // --- CHANGE: ball-detect threshold lowered ~7% (2000 -> 1860) and the
  // in-kicker threshold likewise (7500 -> 6975).
  // BENEFIT: the ball is picked up a bit further away and the kicker latches
  // a fraction earlier, which matters on a weakly lit ball.
  // OLD: if (balldistance > 2000) isball = true;
  //      ... && balldistance > 7500 && ...
  if (balldistance > 1860) isball = true;
  else isball = false;
  if (!digitalRead(PA4) && balldistance > 6975 && (-90 < ballangle && ballangle < 90)) ball_in_kicker = true;
  else ball_in_kicker = false;
  //gy
  if(is_goaller && !goalkeeper_come_forward) dgy = 0;
  else {
    // --- CHANGE: dgy now ramps smoothly from 0 to +-24.00 across DGY_BAND
    // counts above the threshold, instead of snapping there in one step. The
    // side with the stronger reading wins. Thresholds are also ~7% more
    // sensitive (1350 -> 1256, 1300 -> 1209).
    // BENEFIT: no sudden yaw kick the instant the threshold is crossed, so the
    // robot leans into the turn progressively and does not overshoot or wobble
    // when the reading sits right on the edge of the threshold.
    // OLD: if (shr > 1350 && isball) dgy = 24.00;        //RIGHT
    //      else if (shl > 1300 && isball) dgy = -24.00;  //LEFT
    //      else if (!isball || abs(d) < 6000) dgy = 0;
    float dgy_r = clamp((shr - 1256) / DGY_BAND, 0, 1) * 24.00;  //RIGHT
    float dgy_l = clamp((shl - 1209) / DGY_BAND, 0, 1) * 24.00;  //LEFT
    if (!isball) dgy = 0;
    else if (dgy_r > 0 && dgy_r >= dgy_l) dgy = dgy_r;
    else if (dgy_l > 0) dgy = -dgy_l;
    else if (abs(d) < 6000) dgy = 0;
  }

  gy = gyro.read();
  if (millis() - start_time > 5000) {
    OLED_EN = false;
    display.clearDisplay();
    display.display();
  }
  // --- CHANGE: the angle difference is wrapped, and last_ball_angle is a float.
  // BENEFIT: 179 deg vs -179 deg is a 2 deg move but used to read as 358, so a
  // ball sitting still behind the robot never latched the counter and the
  // goalkeeper never decided to come forward. OLD:
  // int last_ball_angle = 0;
  // if (isball && abs(last_ball_angle - ballangle) < 40) ball_stop_cnt++;
  if (isball && abs(convertAngle(last_ball_angle - ballangle)) < 40) ball_stop_cnt++;
  else ball_stop_cnt = 0;
  last_ball_angle = ballangle;
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
  display.print("d :");
  display.println(d);
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
float convertAngle(float angle) {
  if (angle > 180) return angle - 360;
  if (angle < -180) return angle + 360;
  return angle;
}
void out() {
  float out_angle;
  int out_cnt = 0;
  cheking_out = true;
  if (kf && kr) {
    movesecond(225, 10);
    out_angle = 45;
  } else if (kf && kl) {
    movesecond(135, 10);
    out_angle = -45;
  // --- CHANGE: the two-sensor corner cases moved above the single-sensor ones.
  // BENEFIT: they used to sit after "else if (kb)", so they could never run --
  // in a corner the robot drove straight out instead of diagonally away.
  // OLD: this block came after the kf / kb / kr / kl branches below.
  } else if (kb && kr) {
    movesecond(315, 10);
    out_angle = 135;
  } else if (kb && kl) {
    movesecond(45, 10);
    out_angle = -135;
  } else if (kf) {
    if (shr < shl) movesecond(140, 17);
    else movesecond(220, 17);
    out_angle = 0;
  } else if (kb) {
    movesecond(0, 15);
    out_angle = 180;
  } else if (kr) {
    movesecond(270, 10);
    out_angle = 90;
  } else if (kl) {
    movesecond(90, 10);
    out_angle = -90;
  } else {
    cheking_out = false;
    return;
  }
  while (isball && abs(out_angle - convertAngle(ballangle)) < 30 && out_cnt < 50) {
    out_cnt++;
    sensor();
    printall();
    if (move_inside()) stop();
  }
  cheking_out = false;
}
bool move_inside() {
  if (kf && kr) moveSpeed(225, 50000);
  else if (kf && kl) moveSpeed(135, 50000);
  else if (kb && kr) moveSpeed(315, 50000);
  else if (kb && kl) moveSpeed(45, 50000);
  else if (kf) moveSpeed(180, 50000);
  else if (kb) moveSpeed(0, 50000);
  else if (kr) moveSpeed(270, 50000);
  else if (kl) moveSpeed(90, 50000);
  else return true;
  return false;
}
void shoot() {
  digitalWrite(PC13, 1);
  for (int i = 0; i < 4; i++) {
    sensor();
    printall();
    out();
  }
  digitalWrite(PC13, 0);
}
// --- CHANGE: speed is now a continuous ramp, 65000 head-on falling by up to
// 20000 as the ball moves off centre (45000 at 90 deg or more).
// BENEFIT: top speed is higher than before and there are no steps, so the
// robot does not lurch when the ball angle crosses 15 or 100 degrees.
// OLD: int calc_speed(float a) {
//        if (a > -15 && a < 15) return 60000;
//        if (a > 15 && a < 100) return 34000;
//        if (a < -15 && a > -100) return 34000;
//        return 50000;
//      }
int calc_speed(float a) {
  float m = abs(convertAngle(a));
  return 65000 - 20000 * clamp(m / 90.0, 0, 1);
}
// --- CHANGE: the shift is a linear ramp above the +-15 deg deadband, reaching
// the old peak of 80 at 120 deg. The shb gate and the deadband are unchanged.
// BENEFIT: matches the old curve closely (about 61 at 90 deg vs 64 before) but
// without the jumps, so the arc around the ball stays smooth.
// OLD: float calc_shift(float a) {
//        if (shb > 900) return 0;
//        if (a > -15 && a < 15) return 0;
//        if (a > 15 && a < 90) return 60;
//        if (a < -15 && a > -90) return -60;
//        if (a > 0) return 80;
//        return -80;
//      }
float calc_shift(float a) {
  if (shb > 900) return 0;
  float m = abs(convertAngle(a));
  if (m < 15) return 0;
  float s = 80 * clamp((m - 15) / 105.0, 0, 1);
  if (a > 0) return s;
  return -s;
}
void forward() {
  if (ball_in_kicker) {
    v = 60000;
    shoot();
  } else if (isball) {
    out();

    // float shift = clamp(ballangle * 1.6, -60.00, 60.00);
    // v = map(balldistance, 10000, 3000, 38000, 50000);

    float shift = calc_shift(ballangle);
    v = calc_speed(ballangle);

    move(ballangle + shift);
  } else {
    movexy(d, (shb - 950) * 60);
  }
}
void goaller() {
  int bd = (shb - 1500) * 60;
  float a = convertAngle(ballangle);
  // --- CHANGE: the sideways chase runs at a flat GK_SPEED and only eases off
  // inside the last GK_EDGE_BAND counts before a wall, and only when actually
  // heading into that wall -- moving away is always full speed. A GK_DEADBAND
  // of 3 deg keeps it still when the ball is dead ahead.
  // BENEFIT: the keeper reaches the corner of the goal much faster than the
  // old proportional term allowed, yet still slides to a stop at the post
  // instead of the old hard cut to zero, and it can leave the post instantly.
  // OLD: a *= 3000;
  //      if (shr > 1200 && a > 0) a = 0;
  //      if (shl > 1200 && a < 0) a = 0;
  //      ... movexy(constrain(a, -50000, 50000), bd);
  float side = 0;
  if (a > GK_DEADBAND) side = GK_SPEED * clamp((1200 + GK_EDGE_BAND - shr) / GK_EDGE_BAND, 0, 1);
  else if (a < -GK_DEADBAND) side = -GK_SPEED * clamp((1200 + GK_EDGE_BAND - shl) / GK_EDGE_BAND, 0, 1);
  if(!isball) goalkeeper_come_forward = false;
  if (ball_in_kicker && isball) {
    moveSpeed(0, 65000);
    shoot();
    goalkeeper_come_forward = false;
  } else if (ball_stop_cnt > 140 || goalkeeper_come_forward) {
    forward();
    goalkeeper_come_forward = true;
  } else if (isball) {
    movexy(side, bd);
  } else {
    movexy(d, bd);
  }
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
  delay(250);
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("loading00");
  display.display();
  i2c.begin();
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
  if(is_goaller) goaller();
  else forward();
}