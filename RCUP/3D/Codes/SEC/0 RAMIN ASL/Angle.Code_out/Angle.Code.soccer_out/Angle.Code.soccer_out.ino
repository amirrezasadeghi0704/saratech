#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
#define ball_in_kicker !digitalRead(PA4)
float  v;
float ballangle;
float balldistance;
bool isball;
int counter = 0;
int buff[8];
float gy = 0;
int shb, shl, shr, d;
int kf, kr, kb, kl;
int kf_s, kr_s, kb_s, kl_s;
int ldr_sens = 300;
void motor(int ml1, int ml2, int mr2, int mr1) {
  ml1 += gy * 200;
  ml2 += gy * 200;
  mr2 += gy * 200;
  mr1 += gy * 200;
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
void movexy(float vx, float vy) {
  motor(vy + vx, vy - vx, -vy - vx, -vy + vx);
}
void sensor() {
  shb = analogRead(PA1);
  shr = analogRead(PA2);
  shl = analogRead(PA3);
  d = (shl - shr) * 30;
  kf = analogRead(PA5) - kf_s;
  kr = analogRead(PA6) - kr_s;
  kb = analogRead(PA7) - kb_s;
  kl = analogRead(PB0) - kl_s;
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
  }
  digitalWrite(PC13, 0);
}
void printall() {
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
  ///kaf
  display.print("KF: ");
  display.println(kf);
  display.print("KR: ");
  display.println(kr);
  display.print("KB: ");
  display.println(kb);
  display.print("KL: ");
  display.println(kl);

  display.drawCircle(93, 32, 20, WHITE);
  if (isball) display.fillCircle(93 + sin(radians(ballangle)) * 25, 32 - cos(radians(ballangle)) * 25, 2, WHITE);
  display.display();
}
void movesecond(int m , int n) {
  sensor();
  for (int i = 0; i < n; i++) {
    v = 50000;
    move (m);
    sensor();
    printall();
  }
}
void stop() {
  motor(0, 0, 0, 0);
}
void out() {
  int cnt = 0;
  /////RIGHT
  if (kr > ldr_sens)
  {
    movesecond(-90, 10);
    while (ballangle > 0 && isball && cnt < 30)
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
    movesecond(90, 10);
    while (ballangle < 0 && isball && cnt < 30)
    {
      sensor();
      printall();
      stop();
      cnt++;
    }
  }
  /////BACK
  else if (kb > ldr_sens)
  {
    movesecond(0, 17);
    while ((ballangle >= 90 || ballangle <= -90) && isball && cnt < 30)
    {
      sensor();
      printall();
      stop();
      cnt++;
    }
  }
  /////FRONT
  else if (kf > ldr_sens && cnt < 30)
  {
    movesecond(180, 17);
    while (ballangle <= 90 && ballangle >= -90  && isball && cnt < 30)
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
  stop();
  delay(250);
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("loading...");
  display.display();
  //gy
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54); //on
  delay(700);
  Serial1.write(0xA5);
  Serial1.write(0x55); //ofset
  delay(700);
  Serial1.write(0xA5);
  Serial1.write(0x52); //send data


  kf_s = analogRead(PA5);
  kr_s = analogRead(PA6);
  kb_s = analogRead(PA7);
  kl_s = analogRead(PB0);

}
void loop() {
  /////////////////////////////////////////////////
  sensor();
  printall();
  if (ball_in_kicker) {
    shoot();
  }
  else if (isball) {
    out();
    float shift = clamp(ballangle * 1.1, -60, 60);
    v = map(balldistance, 10000, 3000, 38000, 55000);
    move(ballangle + shift);
  }
  else movexy(d, (shb - 1000) * 50);

}
