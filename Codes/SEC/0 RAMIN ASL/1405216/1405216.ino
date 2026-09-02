#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
#define ball_in_kicker !digitalRead(PA4)
float  v , vx;
float shift;
float ballangle;
float balldistance;
bool isball;
int counter = 0;
int buff[8];
float gy = 0, dgy = 0;
int shb, shl, shr, d;
int kf, kr, kb, kl;
int kf_s, kr_s, kb_s, kl_s;
int ldr_sens = 450;
bool OLED_EN = true;
int start_time = 0;

void motor(int ml1, int ml2, int mr2, int mr1) {
  float correction = gy - dgy;
  ml1 += correction * 200;
  ml2 += correction * 200;
  mr2 += correction * 200;
  mr1 += correction * 200;

  if (ml1 > 65535)  ml1 = 65535;
  if (ml1 < -65535) ml1 = -65535;
  if (ml2 > 65535)  ml2 = 65535;
  if (ml2 < -65535) ml2 = -65535;
  if (mr1 > 65535)  mr1 = 65535;
  if (mr1 < -65535) mr1 = -65535;
  if (mr2 > 65535)  mr2 = 65535;
  if (mr2 < -65535) mr2 = -65535;
  /////////////////  MR1
  if (mr1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, mr1);
  }
  else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, mr1 + 65535);
  }
  /////////////////  MR2
  if (mr2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, mr2);
  }
  else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, mr2 + 65535);
  }
  /////////////////  ML1
  if (ml1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PB9, ml1);
  }
  else {
    digitalWrite(PB15, 1);
    pwmWrite(PB9, ml1 + 65535);
  }
  /////////////////  ML2
  if (ml2 > 0) {
    digitalWrite(PB14, 0);
    pwmWrite(PB8, ml2);
  }
  else {
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
int pos(int val) {
  if (val < 0) return 0;
  return val;
}
/// Spinning shot // OLED
void sensor() {
  kf = pos(analogRead(PA5) - kf_s);
  kr = pos(analogRead(PA6) - kr_s);
  kb = pos(analogRead(PA7) - kb_s);
  kl = pos(analogRead(PB0) - kl_s);
  //if ((kf > ldr_sens || kr > ldr_sens || kb > ldr_sens || kl > ldr_sens) && isball) return;
  shb = analogRead(PA1);
  shr = analogRead(PA2);
  shl = analogRead(PA3);
  d = (shl - shr) * 30;

  ////////////////////////////// TSOP angle
  float sumx = 0 , sumy = 0;
  for (int i = 0 ; i < 16; i++) {
    digitalWrite(PA8, (i / 1) % 2);
    digitalWrite(PB1, (i / 2) % 2);
    digitalWrite(PC14, (i / 4) % 2);
    digitalWrite(PC15, (i / 8) % 2);
    float sensorangle = radians(i * 22.5);
    float value = 4095 - analogRead(PA0);
    sumx += value * cos(sensorangle);
    sumy += value * sin(sensorangle);
  }
  ballangle = degrees(atan2(sumy, sumx));
  balldistance = sqrt(pow(sumx, 2) + pow(sumy, 2));
  if (balldistance > 2000) isball = true;
  else isball = false;
  //gy
  while (Serial1.available()) {
    buff[counter] = Serial1.read();
    if (counter == 0 && buff[0] != 0xAA) break;
    counter++;
    if (counter == 8) {
      counter = 0;
      if (buff[0] == 0xAA && buff[7] == 0x55) {
        gy = (int16_t) (buff[1] << 8 | buff[2]) / 100.00;
      }
    }
  }
  if (shr > 1150 ) dgy = 27; // end
  else if (shl > 1350 ) dgy = -30;
  else if (!isball || abs(d) < 8000) dgy = 0;
  //if (millis() - start_time > 10000) OLED_EN = false;
}
float clamp(float val, float min_shift, float max_shift) {
  if (val > max_shift)return max_shift;
  if (val < min_shift)return min_shift;
  return val;
}
void shoot() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(PC13, 1);
    sensor();
    printall();
    out();
  }
  digitalWrite(PC13, 0);
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
  //    display.print("d :");
  //    display.println(d);

  ///kaf
  //    display.print("KF: ");
  //    display.println(kf);
  //    display.print("KR: ");
  //    display.println(kr);
  //    display.print("KB: ");
  //    display.println(kb);
  //    display.print("KL: ");
  //    display.println(kl);

  display.drawCircle(93, 32, 20, WHITE);
  if (isball) display.fillCircle(93 + sin(radians(ballangle + shift)) * 25, 32 - cos(radians(ballangle + shift)) * 25, 2, WHITE);
  display.display();
}
void movesecond(int m , int n) {
  sensor();
  if (!OLED_EN) n *= 30;
  for (int i = 0; i < n; i++) {
    moveSpeed(m, 50000);
    sensor();
    printall();
  }
}
void stop() {
  motor(0, 0, 0, 0);
}
void out() {
  int cnt = 0;
  int out_timeout = 20;
  int out_sec = 15;
  if (!OLED_EN) out_timeout *= 30;
  /////RIGHT
  if (kr > ldr_sens)
  {

    movesecond(-90, out_sec);
    while (ballangle > 0 && isball && cnt < out_timeout)
    {
      sensor();
      printall();
      stop();
      cnt++;
    }
  }
  /////LEFT
  else if (kl > ldr_sens)
  {
    // dgy = 0 ;
    //  sensor();
    movesecond(90, out_sec);
    while (ballangle < 0 && isball && cnt < out_timeout)
    {
      sensor();
      printall();
      stop();
      cnt++;
    }
  }
  /////BACK
  else if (kb > ldr_sens) {
    //dgy = 0 ;
    // sensor();
    movesecond(0, out_sec);
    while ((ballangle >= 90 || ballangle <= -90) && isball && cnt < out_timeout)
    {
      sensor();
      printall();
      stop();
      cnt++;
    }
  }
  /////FRONT
  else if (kf > ldr_sens)
  {
    // dgy = 0 ;
    //sensor();
    if (shl > 1400) movesecond(120, out_sec * 1.3);
    else if (shr > 1400) movesecond(-120, out_sec * 1.3);
    else movesecond(180, out_sec * 1.3);

    while ((ballangle <= 90 && ballangle >= -90)  && isball && cnt < out_timeout)
    {
      sensor();
      printall();
      stop();
      cnt++;
    }
  }
}
void setup() {
  ////////////////////
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);
  //////////////////
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
  ///////////////////////
  pinMode(PA8, OUTPUT);
  pinMode(PB1, OUTPUT);
  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);
  pinMode(PC13, OUTPUT);
  ///////////////////////
  delay(100);
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("loading.26k");
  display.display();
  //gy
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54); //on
  delay(750);
  Serial1.write(0xA5);
  Serial1.write(0x55); //ofset
  delay(750);
  Serial1.write(0xA5);
  Serial1.write(0x52); //send data



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
  /////////////////////////////////////////////////
  sensor();
  printall();
  if (ball_in_kicker && isball) {
    shoot();
  }
  else if (isball) {
    out();
    float shift = clamp(ballangle * 2 , -60, 60 );
    v = map(balldistance, 10000, 3000, 50000, 60000);
    // v = map(abs(ballangle), 0, 180, 30000, 60000);
    move(ballangle + shift);
  }
  else movexy(d, (shb - 1050) * 40);

}
