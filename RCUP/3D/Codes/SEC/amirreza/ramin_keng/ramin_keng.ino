#include <TDAxis12.h>
#include <Wire.h>
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);
#define MAX_MOTOR 65000
float ballangle;
float balldistance;
float sensorangle;
float gy = 0, dgy = 0;
float v, vx;
float value;
float shift;
int shb, shl, shr, d;
int kf_s, kr_s, kb_s, kl_s;
int ldr_sens = 300;
int start_time = 0;
bool kf, kr, kb, kl;
bool isball;
bool ball_in_kicker = false;
bool OLED_EN = true;
bool cheking_out = false;
float out_angle;          // direction the last escape ran toward
bool is_corner = false;   // set by corner() when one of the four corners is on

void motor(int ml1, int ml2, int mr2, int mr1) {
  float correction = clamp(gy - dgy, -90, 90);

  ml1 += (correction * 200);
  ml2 += (correction * 200);
  mr2 += (correction * 200);
  mr1 += (correction * 200);
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
    moveSpeed(m, 52000);
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
/// Spinning shot // OLED
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
  if (balldistance > 2000) isball = true;
  else isball = false;
  if (!digitalRead(PA4) && balldistance > 7500 && (-90 < ballangle && ballangle < 90)) ball_in_kicker = true;
  else ball_in_kicker = false;
  //gy
  if (shr > 1350 && isball) dgy = 23.00;        //RIGHT
  else if (shl > 1300 && isball) dgy = -21.75;  //LEFT
  else if (!isball || abs(d) < 6000) dgy = 0;

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
// void out() {
//   int cnt = 0;
//   int out_timeout = 20;
//   int out_sec = 10;
//   // if (!OLED_EN) out_timeout *= 30;
//   /////RIGHT
//   if (kr) {
//     movesecond(-90, out_sec);
//     while (ballangle > 0 && isball && cnt < out_timeout) {
//       sensor();
//       printall();
//       stop();
//       cnt++;
//     }
//   }
//   /////LEFT
//   else if (kl) {
//     movesecond(90, out_sec);
//     while (ballangle < 0 && isball && cnt < out_timeout) {
//       sensor();
//       printall();
//       stop();
//       cnt++;
//     }
//   }
//   /////BACK
//   // else if (kb) {
//   //   delay(25);
//   //   while (1000 <= shb && isball && abs(d) < 47500 && (-90 <= ballangle <= 90)) {
//   //     sensor();
//   //     move(ballangle);
//   //   }
//   else if (kb) {
//     movesecond(0, out_sec);
//     while ((ballangle >= 90 || ballangle <= -90) && isball && cnt < out_timeout) {
//       sensor();
//       printall();
//       stop();
//       cnt++;
//     }
//   }
//   //}
//   /////FRONT
//   else if (kf) {
//     // if (shl > 1400) movesecond(120, out_sec * 1.3);
//     // else if (shr > 1400) movesecond(-120, out_sec * 1.3);
//     movesecond(180, out_sec * 1.5);
//     while ((ballangle <= 90 && ballangle >= -90) && isball && cnt < out_timeout) {
//       sensor();
//       printall();
//       stop();
//       cnt++;
//     }
//   }
// }

float convertAngle(float angle) {
  if (angle > 180) return angle - 360;
  if (angle < -180) return angle + 360;
  return angle;
}
// The four corner combinations, pulled out of out() so a corner is always
// tested before any single edge. Previously kb && kr and kb && kl sat after
// the bare kb branch, so they could never run and a back corner escaped as if
// it were a plain back line.
void corner() {
  is_corner = true;
  if (kf && kr) {
    movesecond(225, 10);
    out_angle = 45;
  } else if (kf && kl) {
    movesecond(135, 10);
    out_angle = -45;
  } else if (kb && kr) {
    movesecond(315, 10);
    out_angle = 135;
  } else if (kb && kl) {
    movesecond(45, 10);
    out_angle = -135;
  } else {
    is_corner = false;
  }
}
void out() {
  int out_cnt = 0;
  cheking_out = true;
  corner();
  if (!is_corner) {
    if (kf) {
      if (shr < shl) movesecond(135, 17);
      else movesecond(225, 17);
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
  if (kf && kr) moveSpeed(225, 52000);
  else if (kf && kl) moveSpeed(135, 52000);
  else if (kb && kr) moveSpeed(315, 52000);
  else if (kb && kl) moveSpeed(45, 52000);
  else if (kf) moveSpeed(180, 52000);
  else if (kb) moveSpeed(0, 52000);
  else if (kr) moveSpeed(270, 52000);
  else if (kl) moveSpeed(90, 52000);
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
  if (ball_in_kicker) {
    v = 62000;
    shoot();
  } else if (isball) {
    out();
    float shift = clamp(ballangle * 1.6, -60.00, 60.00);
    if(shb > 900) shift = 0;
    v = map(balldistance, 10000, 3000, 40000, 52000);
    // v = map(abs(ballangle), 0, 180, 38000, 52000);
    if (-30 <= ballangle && ballangle <= -5) {  
      move(ballangle);
    } else {
      move(ballangle + shift);
    }
  } else {
    movexy(d, (shb - 950) * 60);
  }
}