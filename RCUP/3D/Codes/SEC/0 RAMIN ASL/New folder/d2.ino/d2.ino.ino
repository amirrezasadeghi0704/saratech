#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

// ================== تنظیمات قابل تنظیم ==================
int motorPower = 65000;  // افزایش قدرت برای حمله تهاجمی‌تر
int motorTurnSpeed = 55000;
int tsopMin = 0, tsopNum = 0;
int heading = 0, gy = 0, d = 0;
int SHL, SHR, SHB;
int buff[8], counter = 0;

// آستانه‌های بهینه
#define TSOP_THRESHOLD 3800
#define TSOP_FRONT_MIN 6   // محدوده جلو گسترده‌تر
#define TSOP_FRONT_MAX 10
#define HEADING_SHOOT_MIN -20  // محدوده شوت گسترده‌تر
#define HEADING_SHOOT_MAX 20
#define HEADING_TOLERANCE 8

// آستانه‌های شارپ
#define SHARP_THRESHOLD 1500  // آستانه تشخیص دیوار
#define SHARP_MIN_DISTANCE 800  // حداقل فاصله از دروازه
#define SHARP_SAFE_DISTANCE 1000  // فاصله امن برای بازگشت

// ----------- سنسورهای LDR ---------------
#define LDR_FRONT   PA4
#define LDR_BACK    PA5
#define LDR_LEFT    PA6
#define LDR_RIGHT   PA7
int LDR_Threshold = 2000;

// ----------- شوتر و اسپین‌بک ------------
#define SHOOTER_PIN PB4
#define SPINBACK_PIN PB5
#define DRIBBLER_SPEED 230  // دریبلر قوی‌تر برای مهاجم

// ----------- حالت‌ها و تایمرها ---------------
enum RobotState { SEARCHING, APPROACHING, ATTACKING, SHOOTING, RETURNING };
RobotState currentState = SEARCHING;
unsigned long lastShootTime = 0;
unsigned long stateChangeTime = 0;
#define SHOOT_COOLDOWN 800
bool ballDetected = false;
int ballLostCount = 0;
unsigned long lastReturnCheck = 0;

// ================== کنترل موتور پیشرفته ==================
void motor(int ML1, int ML2, int MR2, int MR1) {
  ML1 = constrain(ML1, -65535, 65535);
  ML2 = constrain(ML2, -65535, 65535);
  MR2 = constrain(MR2, -65535, 65535);
  MR1 = constrain(MR1, -65535, 65535);
  
  // تصحیح ژایرو نرم‌تر
  int correction = gy / 12;
  ML1 -= correction; ML2 -= correction;
  MR1 += correction; MR2 += correction;
  
  digitalWrite(PB12, (MR1 <= 0));
  pwmWrite(PB6, (MR1 > 0) ? MR1 : MR1 + 65535);
  digitalWrite(PB13, (MR2 <= 0));
  pwmWrite(PB7, (MR2 > 0) ? MR2 : MR2 + 65535);
  digitalWrite(PB14, (ML2 <= 0));
  pwmWrite(PB8, (ML2 > 0) ? ML2 : ML2 + 65535);
  digitalWrite(PB15, (ML1 <= 0));
  pwmWrite(PB9, (ML1 > 0) ? ML1 : ML1 + 65535);
}

// ================== خواندن سنسورها ==================
void readSensors() {
  tsopMin = 4095;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PA8,  (i >> 0) & 1);
    digitalWrite(PB1,  (i >> 1) & 1);
    digitalWrite(PC14, (i >> 2) & 1);
    digitalWrite(PC15, (i >> 3) & 1);
    delayMicroseconds(10);
    int v = analogRead(PA0);
    if (v < tsopMin) { 
      tsopMin = v; 
      tsopNum = i; 
    }
  }
  
  // قطب‌نما با تایم‌اوت
  Serial1.write(0xA5); Serial1.write(0x51);
  unsigned long timeout = millis() + 50;
  while (millis() < timeout) {
    if (Serial1.available()) {
      buff[counter] = Serial1.read();
      if (counter == 0 && buff[0] != 0xAA) break;
      counter++;
      if (counter == 8) {
        counter = 0;
        if (buff[0] == 0xAA && buff[7] == 0x55)
          heading = (int16_t)(buff[1] << 8 | buff[2]) / 100.0;
        break;
      }
    }
  }
  gy = heading * 190;
  
  SHB = analogRead(PA1);
  SHR = analogRead(PA2);
  SHL = analogRead(PA3);
  d = (SHR - SHL) * 15;
}

bool isOutOfField() {
  static bool wasOut = false;
  int frontVal = analogRead(LDR_FRONT);
  int backVal = analogRead(LDR_BACK);
  int leftVal = analogRead(LDR_LEFT);
  int rightVal = analogRead(LDR_RIGHT);
  
  bool isOut = (frontVal > LDR_Threshold || backVal > LDR_Threshold ||
                leftVal > LDR_Threshold || rightVal > LDR_Threshold);
  
  if (isOut) {
    wasOut = true;
    // بازگشت هوشمند از خط
    if (frontVal > LDR_Threshold) motor(-50000, -50000, 50000, 50000);
    else if (backVal > LDR_Threshold) motor(50000, 50000, -50000, -50000);
    else if (leftVal > LDR_Threshold) motor(40000, 40000, -40000, -40000);
    else if (rightVal > LDR_Threshold) motor(-40000, -40000, 40000, 40000);
    delay(150);
  } else if (wasOut && frontVal < LDR_Threshold - 200) {
    wasOut = false;
  }
  return wasOut;
}

// ================== حرکت‌های پیشرفته ==================
void stopRobot() { 
  motor(0, 0, 0, 0); 
}

void moveTowardBall() {
  if (tsopNum == 0 || tsopNum == 15) {
    // توپ کاملاً سمت راست یا چپ
    if (tsopNum == 0) motor(motorTurnSpeed, motorTurnSpeed, -motorTurnSpeed, -motorTurnSpeed);
    else motor(-motorTurnSpeed, -motorTurnSpeed, motorTurnSpeed, motorTurnSpeed);
  }
  else if (tsopNum >= 1 && tsopNum <= 5) {
    // توپ جلو-راست → حرکت مورب با قوت بیشتر
    motor(motorPower, motorPower * 0.3, -motorPower, -motorPower * 0.3);
  }
  else if (tsopNum >= 6 && tsopNum <= 10) {
    // توپ جلو → حرکت مستقیم با حداکثر قدرت
    motor(motorPower, motorPower, -motorPower, -motorPower);
  }
  else if (tsopNum >= 11 && tsopNum <= 14) {
    // توپ جلو-چپ → حرکت مورب
    motor(motorPower * 0.3, motorPower, -motorPower * 0.3, -motorPower);
  }
}

void performShoot() {
  if (millis() - lastShootTime > SHOOT_COOLDOWN) {
    // شوت قدرتمند
    digitalWrite(SHOOTER_PIN, HIGH);
    delay(200);
    digitalWrite(SHOOTER_PIN, LOW);
    lastShootTime = millis();
    
    // عقب‌گرد کوتاه بعد از شوت
    motor(-30000, -30000, 30000, 30000);
    delay(150);
    stopRobot();
  }
}

void searchForBall() {
  // جستجوی چرخشی هوشمند
  static int searchDirection = 1;
  if (millis() - stateChangeTime > 2000) {
    searchDirection *= -1;
    stateChangeTime = millis();
  }
  
  int searchSpeed = 45000;
  if (searchDirection > 0) {
    motor(searchSpeed, searchSpeed, -searchSpeed, -searchSpeed);
  } else {
    motor(-searchSpeed, -searchSpeed, searchSpeed, searchSpeed);
  }
}

// بررسی نیاز به بازگشت به موقعیت اولیه
bool shouldReturnToStart() {
  // هر 3 ثانیه یکبار چک کن
  if (millis() - lastReturnCheck < 3000) return false;
  lastReturnCheck = millis();
  
  // اگر شارپ عقب دیوار دروازه حریف را می‌بیند = خیلی جلو رفته
  if (SHB > SHARP_THRESHOLD) return true;
  
  // یا اگر heading نشان می‌دهد خیلی به سمت دروازه حریف رفته
  if (abs(heading) < 30) return true;  // رو به جلو زیادی
  
  return false;
}

// بازگشت به موقعیت اولیه (نزدیک مرکز زمین)
void returnToStart() {
  // اگر شارپ چپ یا راست دیوار می‌بیند = به کنار زمین رسیده
  bool nearLeftWall = (SHL > SHARP_THRESHOLD);
  bool nearRightWall = (SHR > SHARP_THRESHOLD);
  
  if (SHB > SHARP_MIN_DISTANCE) {
    // هنوز نزدیک دروازه حریف - عقب بیا
    // تراز کردن در حین عقب‌گرد
    int sharpDiff = SHL - SHR;
    
    if (nearLeftWall) {
      // دیوار چپ - به راست و عقب
      motor(-motorPower * 0.5, 0, motorPower * 0.5, 0);
    } else if (nearRightWall) {
      // دیوار راست - به چپ و عقب
      motor(0, -motorPower * 0.5, 0, motorPower * 0.5);
    } else if (abs(heading - 180) < 20 || abs(heading + 180) < 20) {
      // رو به عقب - مستقیم عقب بیا
      motor(-motorPower * 0.6, -motorPower * 0.6, motorPower * 0.6, motorPower * 0.6);
    } else if (heading > 0 && heading < 180) {
      // چرخش به چپ برای رسیدن به 180 درجه
      motor(-motorPower * 0.4, -motorPower * 0.4, motorPower * 0.4, motorPower * 0.4);
    } else {
      // چرخش به راست برای رسیدن به 180 درجه
      motor(motorPower * 0.4, motorPower * 0.4, -motorPower * 0.4, -motorPower * 0.4);
    }
  } else {
    // به موقعیت مناسب رسیده - برگرد به حالت جستجو
    currentState = SEARCHING;
    stopRobot();
  }
}

// ================== راه‌اندازی ==================
void setup() {
  stopRobot();
  pinMode(PB12, OUTPUT); pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
  pinMode(PB9, PWM); pinMode(PB8, PWM); 
  pinMode(PB7, PWM); pinMode(PB6, PWM);
  pinMode(PA8, OUTPUT); pinMode(PB1, OUTPUT); 
  pinMode(PC14, OUTPUT); pinMode(PC15, OUTPUT);
  
  display.begin(0x2, 0x3c);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0); 
  display.print("STRIKER V2.0");
  display.setCursor(0, 16); 
  display.print("Aggressive Mode");
  display.display();
  
  Serial1.begin(115200);
  Serial1.write(0xA5); Serial1.write(0x54); delay(1000);
  Serial1.write(0xA5); Serial1.write(0x55); delay(1000);
  Serial1.write(0xA5); Serial1.write(0x51);
  
  pinMode(SHOOTER_PIN, OUTPUT);
  pinMode(SPINBACK_PIN, PWM);
  
  display.clearDisplay();
  display.setCursor(0, 0); 
  display.print("LET'S GO!");
  display.display();
  delay(1000);
}

// ================== حلقه اصلی با State Machine ==================
void loop() {
  readSensors();
  
  // نمایش وضعیت
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 200) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("T:"); display.print(tsopNum);
    display.print(" V:"); display.print(tsopMin);
    display.setCursor(0, 12);
    display.print("H:"); display.print(heading);
    display.setCursor(0, 24);
    display.print("SB:"); display.print(SHB);
    display.print(" L:"); display.print(SHL);
    display.setCursor(0, 36);
    const char* states[] = {"SEARCH", "APPROACH", "ATTACK", "SHOOT", "RETURN"};
    display.print(states[currentState]);
    display.display();
    lastDisplayUpdate = millis();
  }
  
  // بررسی خط
  if (isOutOfField()) {
    analogWrite(SPINBACK_PIN, 0);
    currentState = SEARCHING;
    return;
  }
  
  // بررسی نیاز به بازگشت (اولویت متوسط)
  if (shouldReturnToStart() && !ballDetected && currentState != RETURNING) {
    currentState = RETURNING;
  }
  
  // تشخیص توپ
  ballDetected = (tsopMin < TSOP_THRESHOLD);
  
  if (!ballDetected) {
    ballLostCount++;
    if (ballLostCount > 5 && currentState != RETURNING) {
      currentState = SEARCHING;
    }
  } else {
    ballLostCount = 0;
    // اگر توپ دید و در حال بازگشت بود، قطع کن
    if (currentState == RETURNING) {
      currentState = APPROACHING;
    }
  }
  
  // State Machine
  switch (currentState) {
    case SEARCHING:
      analogWrite(SPINBACK_PIN, 0);
      if (ballDetected) {
        currentState = APPROACHING;
        stateChangeTime = millis();
      } else {
        searchForBall();
      }
      break;
      
    case APPROACHING:
      analogWrite(SPINBACK_PIN, DRIBBLER_SPEED);
      moveTowardBall();
      if (tsopNum >= TSOP_FRONT_MIN && tsopNum <= TSOP_FRONT_MAX) {
        currentState = ATTACKING;
      }
      break;
      
    case ATTACKING:
      analogWrite(SPINBACK_PIN, DRIBBLER_SPEED);
      if (heading >= HEADING_SHOOT_MIN && heading <= HEADING_SHOOT_MAX) {
        currentState = SHOOTING;
      } else {
        // تصحیح جهت
        if (heading > HEADING_SHOOT_MAX) {
          motor(-35000, -35000, 35000, 35000);
        } else if (heading < HEADING_SHOOT_MIN) {
          motor(35000, 35000, -35000, -35000);
        }
      }
      break;
      
    case SHOOTING:
      performShoot();
      currentState = SEARCHING;
      break;
      
    case RETURNING:
      analogWrite(SPINBACK_PIN, 0);
      returnToStart();
      break;
  }
}