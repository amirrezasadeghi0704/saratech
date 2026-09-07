#include <TDAxis12.h>
#include <Wire.h>
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);
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
bool OLED_EN = true;
bool cheking_out = false;

void motor(int ml1, int ml2, int mr2, int mr1) {
  float correction = gy - dgy;
  ml1 += correction * 200;
  ml2 += correction * 200;
  mr2 += correction * 200;
  mr1 += correction * 200;
  if (ml1 > 65535) ml1 = 65535;
  if (ml1 < -65535) ml1 = -65535;
  if (ml2 > 65535) ml2 = 65535;
  if (ml2 < -65535) ml2 = -65535;
  if (mr1 > 65535) mr1 = 65535;
  if (mr1 < -65535) mr1 = -65535;
  if (mr2 > 65535) mr2 = 65535;
  if (mr2 < -65535) mr2 = -65535;
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
/// Spinning shot // OLED
void sensor() {
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

  gy = gyro.read();

  if (millis() - start_time > 4000) {
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

  display.drawCircle(94, 32, 20, WHITE);
  display.fillTriangle(
    94 + 15 * sin(radians(gy)), 32 - 15 * cos(radians(gy)),
    94 + 21 * sin(radians(gy + 21)), 32 - 21 * cos(radians(gy + 21)),
    94 + 21 * sin(radians(gy - 21)), 32 - 21 * cos(radians(gy - 21)), BLACK);
  display.drawLine(
    94 + 20 * sin(radians(gy + 20)), 32 - 20 * cos(radians(gy + 20)),
    94 + 12 * sin(radians(gy + 30)), 32 - 12 * cos(radians(gy + 30)), WHITE);
  display.drawLine(
    94 + 20 * sin(radians(gy - 20)), 32 - 20 * cos(radians(gy - 20)),
    94 + 12 * sin(radians(gy - 30)), 32 - 12 * cos(radians(gy - 30)), WHITE);
  display.drawLine(
    94 + 12 * sin(radians(gy + 30)), 32 - 12 * cos(radians(gy + 30)),
    94 + 12 * sin(radians(gy - 30)), 32 - 12 * cos(radians(gy - 30)), WHITE);
  if (isball) display.fillCircle(93 + sin(radians(ballangle)) * 25, 32 - cos(radians(ballangle)) * 25, 2, WHITE);
  display.display();
}
float convertAngle(float angle) {
  if (angle > 180) return angle - 360;
  if (angle < -180) return angle + 360;
  return angle;
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
}
void loop() {
  sensor();
  printall();
  if (isball) {
    float shift = clamp(ballangle * 1.6, -60.00, 60.00);
    v = map(balldistance, 10000, 3000, 38000, 50000);
    if (ballangle >= -30 && ballangle <= -5) {
      move(ballangle);
    } else if (ballangle < -10 || ballangle > 10) {
      move(ballangle + shift);
    }
  } else {
    movexy(d, (shb - 950) * 60);
  }
}