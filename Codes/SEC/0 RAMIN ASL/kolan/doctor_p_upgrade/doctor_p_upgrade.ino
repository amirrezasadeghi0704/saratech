#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);
#define ball_in_kicker !digitalRead(PA4)

float v;
float shift = 0;
float ballangle = 0;
float ballangle_filtered = 0;  // FIX 6: moving average
float balldistance;
bool isball;
int min_speed = 25000, max_speed = 50000;

int counter = 0;
int buff[8];
float gy = 0, prev_gy = 0, dgy = 0;  // FIX 3: dgy محاسبه می‌شه
int shb, shl, shr, d;

// FIX 5: آرایه کالیبراسیون — هر ایندکس یک سنسور TSOP
float tsop_cal[16] = {
  1.00,// 0 jolo
  0.95,// 1 
  0.95,// 2
  1.00,// 3
  1.00,// 4 rast
  1.00,// 5
  1.00,// 6
  1.00,// 7
  1.00,// 8 aghab
  1.00,// 9
  1.00,// 10
  1.00,// 11
  1.00,// 12 chap
  1.00,// 13
  1.10,// 14
  1.10 // 15
};

// ─── motor ────────────────────────────────────────────────
void motor(int ml1, int ml2, int mr2, int mr1) {
  ml1 += gy * 202;
  ml2 += gy * 202;
  mr2 += gy * 202;
  mr1 += gy * 202;
  if (isball) {
    ml1 += dgy;
    ml2 += dgy;
    mr2 += dgy;
    mr1 += dgy;
  }

  ml1 = constrain(ml1, -65535, 65535);
  ml2 = constrain(ml2, -65535, 65535);
  mr1 = constrain(mr1, -65535, 65535);
  mr2 = constrain(mr2, -65535, 65535);

  if (mr1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, mr1);
  } else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, mr1 + 65535);
  }

  if (mr2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, mr2);
  } else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, mr2 + 65535);
  }

  if (ml1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PB9, ml1);
  } else {
    digitalWrite(PB15, 1);
    pwmWrite(PB9, ml1 + 65535);
  }

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

void movexy(float vx, float vy) {
  motor(vy + vx, vy - vx, -vy - vx, -vy + vx);
}

void stop() {
  motor(0, 0, 0, 0);
}

float clamp(float val, float mn, float mx) {
  if (val > mx) return mx;
  if (val < mn) return mn;
  return val;
}

// ─── sensor ───────────────────────────────────────────────
void sensor() {
  shb = analogRead(PA1);
  shr = analogRead(PA2);
  shl = analogRead(PA3);
  d = (shl - shr) * 30;

  
  float sumx = 0, sumy = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8, (i / 1) % 2);
    digitalWrite(PB1, (i / 2) % 2);
    digitalWrite(PC14, (i / 4) % 2);
    digitalWrite(PC15, (i / 8) % 2);
    delayMicroseconds(50);                               
    float value = (4095 - analogRead(PA0)) * tsop_cal[i];  
    sumx += value * cos(radians(i * 22.5));
    sumy += value * sin(radians(i * 22.5));
  }
  ballangle = degrees(atan2(sumy, sumx));
  balldistance = sqrt(sumx * sumx + sumy * sumy);
  isball = (balldistance > 2000);


  ballangle_filtered = ballangle_filtered * 0.6 + ballangle * 0.4;

  while (Serial1.available() >= 8) {
    for (int i = 0; i < 8; i++) buff[i] = Serial1.read();
    if (buff[0] == 0xAA && buff[7] == 0x55) {
      prev_gy = gy;
      gy = (int16_t)(buff[1] << 8 | buff[2]) / 100.00;
      dgy = gy - prev_gy;  
    }
  }
  gy += clamp(d / 4000.0, -25, 25);
}


// ─── display ──────────────────────────────────────────────
void printall() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("BA:");
  display.println(ballangle_filtered);
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
  display.drawCircle(93, 32, 20, WHITE);
  if (isball)
    display.fillCircle(
      93 + sin(radians(ballangle_filtered + shift)) * 25,
      32 - cos(radians(ballangle_filtered + shift)) * 25,
      2, WHITE);
  display.display();
}

// ─── movesecond ───────────────────────────────────────────
void movesecond(int m, int n) {
  sensor();
  for (int i = 0; i < n; i++) {
    move(m);
    sensor();
    printall();
  }
}

// ─── setup ────────────────────────────────────────────────
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
  delay(100);
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("loading");
  display.display();
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54);
  delay(750);
  Serial1.write(0xA5);
  Serial1.write(0x55);
  delay(750);
  Serial1.write(0xA5);
  Serial1.write(0x52);
  kf_s = analogRead(PA5);
  kr_s = analogRead(PA6);
  kb_s = analogRead(PA7);
  kl_s = analogRead(PB0);
}
// ─── loop ─────────────────────────────────────────────────
void loop() {
  sensor();
  printall();

  if (isball) {
    shift = clamp(ballangle_filtered * 1.15, -40.000, 40.000);
    v = clamp(
      (balldistance - 10000.0) * (max_speed - min_speed) / (3000.0 - 10000.0) + min_speed,
      min_speed, max_speed);
    move(ballangle_filtered + shift);
  }
  else {
    shift = 0;
    movexy(d, (shb - 1300) * 50);
  }
}