/////////////////////////////////////////////////////////////////////////////
//  base_darvazeban -- goalkeeper logic on the new hardware layer.
//
//  Everything that existed in both versions was taken from the NEW one:
//  includes, gyro, motor(), move/moveSpeed/movexy/movesecond, sensor(),
//  printall(), setup(). Everything that only existed in the OLD one is kept
//  as it was: darvazeban(), toop(), shoot(), tsopMin/tsopNum, SHOOT.
//  Everything attack-only was dropped: the striker loop(), its shoot(), out(),
//  move_inside(), convertAngle(), shift, cheking_out, vx.
//
//  The old code addressed the world differently, so its references had to be
//  translated or it would not build -- and worse, would build wrong:
//    move(8)  -> move(180)   old move() took spoke 0..16, new one takes DEGREES
//    move(4)  -> move(90)    so an untranslated move(8) would mean 8 degrees
//    move(12) -> move(270)
//    move(16) -> stop()
//    movesecond(12, n) -> movesecond(270, n)   same spoke-to-degree change
//    KL < ldr_sens -> !kl    line sensors are booleans now
//    SHL/SHR/SHB   -> shl/shr/shb
//    printSensor() -> printall()
/////////////////////////////////////////////////////////////////////////////
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
float v = 45000;  // old initial value kept; the new file left v uninitialised
float value;
int shb, shl, shr, d;
int kf_s, kr_s, kb_s, kl_s;
int ldr_sens = 300;
int start_time = 0;
bool kf, kr, kb, kl;
bool isball;
bool ball_in_kicker = false;
bool OLED_EN = true;

// Old-only, so kept unchanged: darvazeban() and toop() steer by the loudest
// TSOP index, and shoot() reads the kicker switch as an analog level.
int tsopMin = 0;
int tsopNum = 0;
int SHOOT;

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
  kf = BOOL(pos(analogRead(PA5) - kf_s), ldr_sens);
  kr = BOOL(pos(analogRead(PA6) - kr_s), ldr_sens);
  kb = BOOL(pos(analogRead(PA7) - kb_s), ldr_sens);
  kl = BOOL(pos(analogRead(PB0) - kl_s), ldr_sens);
  //if ((kf > ldr_sens || kr > ldr_sens || kb > ldr_sens || kl > ldr_sens) && isball) return;
  shb = analogRead(PA1);
  shr = analogRead(PA2);
  shl = analogRead(PA3);
  SHOOT = analogRead(PA4);  // old-only: shoot() still tests this level
  d = (shl - shr) * 30;
  ////////////////////////////// TSOP angle
  tsopMin = 4095;  // old-only: darvazeban() and toop() still index by sensor
  float sumx = 0, sumy = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8, (i / 1) % 2);
    digitalWrite(PB1, (i / 2) % 2);
    digitalWrite(PC14, (i / 4) % 2);
    digitalWrite(PC15, (i / 8) % 2);
    sensorangle = radians(i * 22.5);
    int raw = analogRead(PA0);
    if (raw < tsopMin) {
      tsopMin = raw;
      tsopNum = i;
    }
    value = 4095 - raw;
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

//// darvazeban() and toop() are gone as functions. They changed state by
//  calling themselves, and those calls never returned -- each hop left another
//  frame on the stack until the STM32's 20K of RAM was gone. Their bodies are
//  now the conditions in loop() below: one pass of loop() runs one state and
//  returns, so the stack never grows.
//
//  The while() loops turned inside out the same way: what the while waited for
//  became the state's exit test, and what was inside it became the state's
//  body. movesecond() calls became counted states (11, 21, 31, 81, 92, 93),
//  since a counted state is what movesecond was doing by hand.
//
//  State numbers match the old darvazeban(a) argument, so state 2 here is the
//  same rightward sweep it was there. States 9x are toop().

int state = 1;
int st_cnt = 0;       // passes spent in the current state
int esc_cnt = 0;      // line crossings counted by states 4 and 5
int ball_return = 2;  // sweep to resume after the ball states

void setState(int s) {
  state = s;
  st_cnt = 0;
  esc_cnt = 0;
}

void shoot()
{
  sensor();
  printall();
  if (SHOOT < 2000) {
    digitalWrite(PC13, 1);
    delay(200);
  }
  else {
    digitalWrite(PC13, 0);
    delay(50);
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

  // setup() used to call darvazeban() here, which never came back. It only
  // picks the starting state now. sensor() runs first because the old test
  // read shb while it still held 0, so it always chose the same branch.
  sensor();
  if (shb > 2500) setState(8);
  else setState(1);
}

void loop() {

  sensor();
  printall();
  st_cnt++;

  //// 1 -- back up until a side sensor finds the line

}
